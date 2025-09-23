from config_loader import load_config
cfg, meta = load_config()
print("# Using UAV_ID:", meta["UAV_ID"], "profile:", meta["used_profile"])
import yaml; print(yaml.dump(cfg, sort_keys=False))
