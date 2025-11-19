from data_manager import results
import json

with open('src/lyapunov_adaptive_transformer/lyapunov_adaptive_transformer/config.json', 'r') as config_file:
    config = json.load(config_file)

results()