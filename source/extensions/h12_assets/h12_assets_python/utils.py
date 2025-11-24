import isaaclab.sim as sim_utils
import os

def spawn_asset(usd_path, prim_path):
    prim = sim_utils.spawn_from_usd(
        prim_path=prim_path,
        cfg=sim_utils.UsdFileCfg(usd_path=usd_path)
    )

def get_h12_assets_path():
    return os.getenv("H12_ASSETS_PATH")

def get_dir_usd_file(directory: str):
    for path in os.listdir(directory):
        if ".usd" in path:
            return directory + f"/{path}"
    
if __name__ == "__main__":
    print(get_dir_usd_file("/root/h12_sim_assets/assets/drill"))