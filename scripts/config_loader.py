import os, yaml
from pathlib import Path
def deep_merge(a, b):
    for k, v in (b or {}).items():
        if isinstance(v, dict) and isinstance(a.get(k), dict):
            a[k] = deep_merge(a[k], v)
        else:
            a[k] = v
    return a
def load_config():
    base = {}
    d = Path("config/default.yaml")
    if d.exists(): base = deep_merge(base, yaml.safe_load(d.read_text()))
    uav_id = os.environ.get("UAV_ID") or os.uname().nodename
    p = Path("config/profiles") / f"{uav_id}.yaml"
    if p.exists(): base = deep_merge(base, yaml.safe_load(p.read_text()))
    l = Path("config/local.yaml")
    if l.exists(): base = deep_merge(base, yaml.safe_load(l.read_text()))
    return base, {"UAV_ID": uav_id, "used_profile": p.name if p.exists() else None}
if __name__ == "__main__":
    cfg, meta = load_config()
    print("# Using UAV_ID:", meta["UAV_ID"], "profile:", meta["used_profile"])
    print(yaml.dump(cfg, sort_keys=False))
