#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
from geodesy import utm
import utm
import geodesy
import tf2_ros
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
import tf2_geometry_msgs
from mavros_msgs.msg import PositionTarget, State, Altitude
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from geographic_msgs.msg import GeoPose, GeoPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from scipy.spatial.transform import Rotation as R
import time
import traceback
import asyncio
import json
from sensor_msgs.msg import Joy
import torch

# import LyAT funcs
from . import LyAT
from . import data_manager

class LyapunovAdaptiveTransformer(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('lyapunov_adaptive_transformer')

        self.initialize_lyat()

        # Load park parameters for coordinate transforms
        self.declare_parameters(
            namespace='',
            parameters=[
                # Origin parameters
                ('origin_r', rclpy.parameter.Parameter.Type.DOUBLE),
                ('origin_x', rclpy.parameter.Parameter.Type.DOUBLE),
                ('origin_y', rclpy.parameter.Parameter.Type.DOUBLE),
                ('origin_z', rclpy.parameter.Parameter.Type.DOUBLE),
                ('utm_zone', rclpy.parameter.Parameter.Type.INTEGER),
                ('utm_band', rclpy.parameter.Parameter.Type.STRING),
            ]
        )

        self.origin_r = self.get_parameter('origin_r').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.origin_z = self.get_parameter('origin_z').value
        self.utm_zone = self.get_parameter('utm_zone').value
        self.utm_band = self.get_parameter('utm_band').value
        

        # Check for missing parameters
        if (self.origin_r is None or self.origin_x is None or 
            self.origin_y is None or self.origin_z is None or 
            self.utm_zone is None or self.utm_band is None):
            raise RuntimeError("Missing required origin parameters")
        
        # Converting to quaternion
        self.q_apark_to_utm = euler_to_quaternion(0, 0, -self.origin_r) 

        # State variables  
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.target_position = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.control_input = np.zeros(3)
        self.orientation = 0.0
        self.quaternion = None
        self.global_pose = NavSatFix()
        self.altitude_amsl = -1.0

        # for logging/plotting
        self.step = 1
 

        # State variables - mavros
        self.mavros_state = None
        self.armed = False
        self.offboard_mode = False
        self.takeoff_mode = False

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.vel_pub = self.create_publisher(PositionTarget, 'setpoint_raw/local', qos_profile=qos_profile_sensor_data)
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, 'autonomy_park/pose', self.pose_callback, qos_profile=qos_profile_sensor_data)
        self.vel_sub = self.create_subscription(TwistStamped, 'local_position/velocity_local', self.velocity_callback, qos_profile=qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, 'state', self.state_callback, qos_profile=qos_profile_sensor_data)
        self.altitude_sub = self.create_subscription(Altitude, 'altitude', self.altitude_callback, qos_profile=qos_profile_sensor_data)
        self.global_pos_sub = self.create_subscription(NavSatFix, 'global_position/global', self.global_pose_callback, qos_profile=qos_profile_sensor_data)
        # self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_profile=qos_profile_sensor_data)

        # Service clients
        self.arming_client = self.create_client(CommandBool, 'cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {self.arming_client.srv_name} not available, waiting...')

        self.takeoff_client = self.create_client(CommandTOL, 'cmd/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {self.takeoff_client.srv_name} not available, waiting...')

        self.set_mode_client = self.create_client(SetMode, 'set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {self.set_mode_client.srv_name} not available, waiting...')

       
        self.get_logger().info("Lyapunov Adaptive Transformer Node Initialized")

    def initialize_lyat(self):
         # Load configuration file for simulation
        with open('src/lyapunov_adaptive_transformer/lyapunov_adaptive_transformer/config.json', 'r') as config_file: config = json.load(config_file)
        self.config = config
        self.n_states = self.config['n_states']
        self.tf = self.config['T_final']
        self.dt = self.config['dt']
        self.time_steps = int(self.tf / self.dt)
        
        self.dynamics = LyAT.Dynamics()        

     # ==================== CALLBACK METHODS ====================
    
    def pose_callback(self, msg):
        self.get_logger().info(f"Pose callback triggered: {self.position[0]}, {self.position[1]}, {self.position[2]}")
        # Pose updates (APark)
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        self.quaternion = msg.pose.orientation
        self.orientation = quat_to_yaw(msg.pose.orientation)

    def global_pose_callback(self, msg):
        # Global pose updates (LLA)
        self.global_pose = msg

    def altitude_callback(self, msg):
        # Altitude update (global)
        self.altitude_amsl = msg.amsl
    
    def velocity_callback(self, msg):
        # Velocity update (body-fixed)
        vel_east = msg.twist.linear.x
        vel_north = msg.twist.linear.y
        vel_up = msg.twist.linear.z

        # Convert to apark frame
        self.velocity[0] = math.cos(self.origin_r)*vel_east - math.sin(self.origin_r)*vel_north
        self.velocity[1] = math.sin(self.origin_r)*vel_east + math.cos(self.origin_r)*vel_north
        self.velocity[2] = vel_up
    
    def state_callback(self, msg):
        # Mavros state update
        self.mavros_state = msg
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")
        
    # def joy_callback(self, joy_msg):
    #     # Joy input callback
    #     accel_x = joy_msg.axes[1]
    #     accel_y = joy_msg.axes[0]
    #     accel_z = joy_msg.axes[4]

    #     joy_msg = PositionTarget()
    #     joy_msg.header.stamp = self.get_clock().now().to_msg()
    #     joy_msg.header.frame_id = "base_link"
    #     joy_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        
    #     # Set the type mask to use acceleration only
    #     joy_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | \
    #                    PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | \
    #                    PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
    #                    PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

    #     # Set acceleration values - ensure they're floats
    #     joy_msg.acceleration_or_force.x = math.cos(self.origin_r)*accel_x + math.sin(self.origin_r)*accel_y
    #     joy_msg.acceleration_or_force.y = -math.sin(self.origin_r)*accel_x + math.cos(self.origin_r)*accel_y
    #     joy_msg.acceleration_or_force.z = accel_z

        # log to check control input
        #self.get_logger().info(f"ax: {joy_msg.acceleration_or_force.x}, ay: {joy_msg.acceleration_or_force.y}, az: {joy_msg.acceleration_or_force.z}")

        # Uncomment to enable direct joystick control
        #self.vel_pub.publish(joy_msg)
        
    def get_desired_state(self, t):
        # Common parameters
        height = 2.0  # meters
        omega = 0.2  # rad/s
        
        # Default values
        x_d, y_d, z_d = 0.0, 0.0, height
        vx_d, vy_d, vz_d = 0.0, 0.0, 0.0
        ax_d, ay_d, az_d = 0.0, 0.0, 0.0
        
        # Trajectory 1: Hover at point (0,0,8)
        if self.trajectory == "point":
            # All values are already initialized to hover
            pass
            
        # Trajectory 2: Circle centered at origin
        elif self.trajectory == "circle":
            r = 2.5  # Radius of circle (5 meters)
            
            # Position
            x_d = r * np.cos(omega * t)
            y_d = r * np.sin(omega * t)
            
            # Velocity
            vx_d = -r * omega * np.sin(omega * t)
            vy_d = r * omega * np.cos(omega * t)
            
            
        # Trajectory 3: Figure eight in x-direction
        elif self.trajectory == "figure8":
            a = 5.0  # Half-width of the long side (x-direction)
            b = 2.5  # Half-width of the short side (y-direction)
            
            # Position (figure 8 with major axis along x)
            x_d = a * np.sin(omega * t)
            y_d = b * np.sin(2 * omega * t)
            
            # Velocity
            vx_d = a * omega * np.cos(omega * t)
            vy_d = 2 * b * omega * np.cos(2 * omega * t)

        
        # Update states
        self.target_position[0] = x_d
        self.target_position[1] = y_d
        self.target_position[2] = z_d
        self.target_velocity[0] = vx_d
        self.target_velocity[1] = vy_d
        self.target_velocity[2] = vz_d

        # Broadcast tf for rViz
        tf = TransformStamped()
        # Set the timestamp and frame IDs
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "autonomy_park"  # Parent frame
        tf.child_frame_id = "target_position"  # Child frame
        
        # Set the translation (position)
        tf.transform.translation.x = x_d
        tf.transform.translation.y = y_d
        tf.transform.translation.z = height
        
        # Set the rotation (identity quaternion if no specific orientation)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
    
        # Send the transform
        self.tf_broadcaster.sendTransform(tf)

        return (np.array([x_d, y_d, z_d]), 
                np.array([vx_d, vy_d, vz_d]), 
                np.array([ax_d, ay_d, az_d]))

    
    def compute_control_input(self):
        # u = LyAT.getInput(self.state, t)
        # u: <ax, ay, ax>
        
        return
       

    def send_command(self, vel_x, vel_y, vel_z, yaw=None, yaw_rate=None):
        """Send velocity and attitude command to PX4"""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Determine which commands to ignore based on what's provided
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | \
                    PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | \
                    PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        
        # Add yaw control if provided
        if yaw is not None:
            msg.yaw = yaw + self.origin_r
            msg.type_mask |= PositionTarget.IGNORE_YAW_RATE
        elif yaw_rate is not None:
            msg.yaw_rate = yaw_rate
            msg.type_mask |= PositionTarget.IGNORE_YAW
        else:
            # If neither yaw nor yaw_rate provided, ignore both
            msg.type_mask |= PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        
        # Set velocity values - ensure they're floats
        vel_enu_x = math.cos(self.origin_r)*vel_x + math.sin(self.origin_r)*vel_y
        vel_enu_y = -math.sin(self.origin_r)*vel_x + math.cos(self.origin_r)*vel_y
        
        # Saturate acceleration
        msg.velocity.x, msg.velocity.y, msg.velocity.z = saturate_vector(vel_enu_x, vel_enu_y, vel_z, 2.0)
        
        # log to check control input
        #self.get_logger().info(f"ax: {msg.acceleration_or_force.x}, ay: {msg.acceleration_or_force.y}, az: {msg.acceleration_or_force.z}")
        
        
        self.vel_pub.publish(msg)
        
    async def arm(self):
        """Arm the vehicle"""

        last_request_time = self.get_clock().now()
        
        while rclpy.ok():
            current_time = self.get_clock().now()
            
            if not self.armed and (current_time - last_request_time).nanoseconds > 2e9:
                self.get_logger().info("Trying to arm...")
                req = CommandBool.Request()
                req.value = True
                future = self.arming_client.call_async(req)
                await self.spin_until_future_complete(future)
                
                if future.result().success:
                    self.get_logger().info("Vehicle armed")
                    return True
                last_request_time = self.get_clock().now()
                
            self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)  # Send neutral commands while waiting
            await self.sleep(0.05)
    
    async def set_offboard(self):
        """Set to offboard mode"""
        # Send a few setpoints before starting
        for i in range(100):
            self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)
            await self.sleep(0.05)

        last_request_time = self.get_clock().now()

        
        while rclpy.ok():
            current_time = self.get_clock().now()
            
            if not self.offboard_mode and (current_time - last_request_time).nanoseconds > 2e9:
                self.get_logger().info("Trying to set OFFBOARD mode...")
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                future = self.set_mode_client.call_async(req)
                await self.spin_until_future_complete(future)
                
                if future.result().mode_sent:
                    self.get_logger().info("OFFBOARD mode set")
                    self.offboard_mode = True
                    return True
                last_request_time = self.get_clock().now()
                
            self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)  # Send neutral commands while waiting
            await self.sleep(0.05)
    
    async def takeoff(self, height):
        """Simple takeoff procedure"""
        last_request_time = self.get_clock().now()
        
        while rclpy.ok():
            current_time = self.get_clock().now()

            if not self.takeoff_mode and (current_time - last_request_time).nanoseconds > 2e9:
                self.get_logger().info(f"Trying to takeoff to {height} meters...")

                takeoff_pose = Pose()
                takeoff_pose.position = Point(
                    x = float(self.position[0]),
                    y = float(self.position[1]),
                    z = float(self.position[2]))
                global_pose = self.apark_to_global(apark_pose=takeoff_pose)

                # grab home location
                # self.home_pose.x = takeoff_pose.x
                # self.home_pose.y = takeoff_pose.y
                # self.home_pose.z = 1.5 # offset from the ground

                # convert local takeoff (apark frame) to global (lat/long)
                req = CommandTOL.Request()
                req.min_pitch = 0.0
                req.yaw = quat_to_yaw(quat = global_pose.orientation)
                req.latitude = self.global_pose.latitude
                req.longitude = self.global_pose.longitude
                req.altitude = self.altitude_amsl - self.position[2] + height
                
                self.get_logger().info(f"lat: {req.latitude}, long: {req.longitude}")
                future = self.takeoff_client.call_async(req)
                await self.spin_until_future_complete(future)

                if future.result().success:
                    self.get_logger().info(f"Taking off to {req.altitude} meters.")
                    self.takeoff_mode = True
                    return True
                
            await self.sleep(0.02)

        self.get_logger().info("Finished Taking off")
    
    async def run_trajectory(self):
        self.get_logger().info("Starting trajectory tracking...")
        
        traj_start_time = self.get_clock().now()
        
        controller = LyAT.LyAT_Controller(self.config)
        tracking_errors = []
        control_inputs = []
        
        # Compute control input
        # Progress
        while rclpy.ok(): 
            try:
                t = (self.get_clock().now() - traj_start_time).nanoseconds / 1e9     

                if t > self.tf:
                    self.get_logger().info(f"Reached final time of {self.tf} seconds.")
                    break

                x = torch.tensor([self.position[0], self.position[1], self.position[2],
                                        self.velocity[0], self.velocity[1], self.velocity[2]],
                                    dtype=torch.float32)
                self.get_logger().info(f"position: {self.position[0]}, { self.position[1]}, {self.position[2]}")
                # Convert t to tensor before using it
                t_tensor = torch.tensor(t, dtype=torch.float32)

                xd, xd_dot = LyAT.Dynamics.desired_trajectory(t_tensor)
                u, Phi = controller.parameter_adaptation(x, t_tensor)

                e = x - xd

                # Ensure we have float values
                vx = float(u[0])
                vy = float(u[1])
                vz = float(u[2])
                
                # Send velocity command
                self.send_command(vx, vy, vz, yaw=None, yaw_rate=None)

                # Data Storage
                data_manager.save_state_to_csv(
                    self.step, 
                    t,  
                    x.detach().cpu().numpy(),
                    xd.detach().cpu().numpy(),
                    u.detach().cpu().numpy()
                )
                self.step = self.step + 0.02

                await self.sleep(0.02)
            
            except Exception as e:
                self.get_logger().error(f"Error in control loop: {e}")
                self.get_logger().error(f"Error details: {type(e)}") 
        


            
    
    async def land(self):
        """Simple landing procedure"""
        self.get_logger().info("Landing...")
        
        # Switch to land mode
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        future = self.set_mode_client.call_async(req)
        await self.spin_until_future_complete(future)
        
        if future.result().mode_sent:
            self.get_logger().info("AUTO.LAND mode set")
            
        # Wait for landing
        while self.armed and rclpy.ok():
            await self.sleep(0.5)
            
        self.get_logger().info("Landing complete")
    
    async def sleep(self, seconds):
        """Sleep while still processing callbacks"""
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > seconds:
                break
    
    async def spin_until_future_complete(self, future):
        """Spin until future is complete"""
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return future.result()
    
    async def run_mission(self):
        """Run the complete mission"""
        try:
                # Arm first
                await self.arm()
                
                # Send some neutral commands to stabilize
                for i in range(80):
                    self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)
                    await self.sleep(0.01)
                
                # Take off next
                await self.takeoff(height=8.0)
                
                # Set offboard mode after takeoff
                await self.set_offboard()
                
                # Run trajectory
                await self.run_trajectory()
        except Exception as e:
            self.get_logger().error(f"Error in mission: {e}")
            # Print more details about the error
            self.get_logger().error(traceback.format_exc())

        finally:
            # Land when done or if interrupted
            await self.land()


    def apark_to_global(self, apark_pose):
        # Autonomy park setpoint coordinates
        sp_x = apark_pose.position.x
        sp_y = apark_pose.position.y
        
        # Un-rotate setpoint coordinates
        dx = math.cos(self.origin_r) * sp_x + math.sin(self.origin_r) * sp_y
        dy = -math.sin(self.origin_r) * sp_x + math.cos(self.origin_r) * sp_y
        
        # Convert park coordinates to UTM
        utm_pos = geodesy.utm.UTMPoint()
        utm_pos.zone = self.utm_zone
        utm_pos.band = self.utm_band
        utm_pos.easting = dx + self.origin_x
        utm_pos.northing = dy + self.origin_y
        
        # Convert UTM easting/northing to lat/long
        lat, lon = utm.to_latlon(self.origin_x, self.origin_y, self.utm_zone, self.utm_band)
        global_pos = GeoPoint(
            latitude = lat,
            longitude = lon,
            altitude = self.altitude_amsl
            )
        
        # IMPORTANT: Command altitude is AMSL! (feedback is WGS-84 ellipsoid)
        global_pos.altitude = self.altitude_amsl
        
        # Finally, compute global orientation
        q_utm = multiply_quaternions(q1 = self.q_apark_to_utm, q2 = apark_pose.orientation)
        
        global_pose = GeoPose(
            position = global_pos,
            orientation = q_utm)
        
        return global_pose

def saturate_vector(vec_x, vec_y, vec_z, max_magnitude):
    """
    Saturate a 3D vector while preserving its direction.
    
    Args:
        vec_x, vec_y, vec_z: Vector components
        max_magnitude: Maximum allowed magnitude
        
    Returns:
        Tuple of saturated (x, y, z) components
    """
    # Calculate current magnitude
    magnitude = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
    
    # If magnitude exceeds limit, scale the vector down
    if magnitude > max_magnitude and magnitude > 0:
        scaling_factor = max_magnitude / magnitude
        return (vec_x * scaling_factor, 
                vec_y * scaling_factor, 
                vec_z * scaling_factor)
    else:
        return (vec_x, vec_y, vec_z)

def quat_to_yaw(quat):
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def multiply_quaternions(q1, q2):
    """Multiply two geometry_msgs.msg.Quaternion quaternions in ROS2."""
    # Convert ROS2 Quaternions to scipy Rotation objects
    r2 = R.from_quat([q2.x, q2.y, q2.z, q2.w])
    r1 = R.from_quat([q1.x, q1.y, q1.z, q1.w])

    # Multiply rotations
    r_result = r1 * r2  # Equivalent to quaternion multiplication

    # Convert back to a geometry_msgs Quaternion
    x, y, z, w = r_result.as_quat()
    return Quaternion(x=x, y=y, z=z, w=w)

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (roll, pitch, yaw) to a geometry_msgs.msg.Quaternion."""
    r = R.from_euler('xyz', [roll, pitch, yaw])  # 'xyz' means rotation order
    x, y, z, w = r.as_quat()  # Convert to (x, y, z, w) format
    return Quaternion(x=x, y=y, z=z, w=w)

def main(args=None):
    rclpy.init(args=args)
    
    lyapunov_adaptive_transformer = LyapunovAdaptiveTransformer()
    
    # Create the event loop
    loop = asyncio.get_event_loop()
    
    try:
        # Run the async method in the event loop
        loop.run_until_complete(lyapunov_adaptive_transformer.run_mission())
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        lyapunov_adaptive_transformer.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == '__main__':
    main()