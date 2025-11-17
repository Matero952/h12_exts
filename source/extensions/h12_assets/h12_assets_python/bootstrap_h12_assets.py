import os
import json
import subprocess

ASSETS_ENV_VAR = "H12_ASSETS_PATH"

def bootstrap_assets():
    assets_env_var = os.getenv(ASSETS_ENV_VAR)
    assert assets_env_var is not None, f"Set '{ASSETS_ENV_VAR}' to the h12_sim_assets directory, if it is not already solved, set this variable to: desired_host_path/h12_sim_assets"
    if assets_env_var[-1] != "/":
        assets_env_var += "/"
    assets_exist = h_ensure_assets_exist(assets_env_var)


def h_ensure_assets_exist(assets_env_path: str):
    with open('asset_manifest.json', 'r') as f:
        data = json.load(f)
        for asset in data['assets']:
            file_base = assets_env_path + "assets/" + asset['name'] + '/'
            # print(f"{file_base=}")
            for field in asset:
                # print(f"{asset[field]=}")
                if field != "name":
                    if "tga" in field:
                        file_base += "textures/"
                    file = file_base + asset[field]
                    try:
                        assert os.path.exists(file), f"{file} does not exist."
                    except AssertionError:
                        print(f"File: {file} does not exist, downloading now.")
                        os.makedirs(os.path.dirname(file), exist_ok=True)
                        get_missing_asset(file, assets_env_path)
                    print(f"File: {file} exists!")

def get_missing_asset(missing_file: str, assets_env_path):
    git_base_url = "https://raw.githubusercontent.com/Matero952/h12_sim_assets/main"
    git_file_path = missing_file.replace("/root/h12_sim_assets", "")
    full_url = git_base_url + git_file_path
    result = subprocess.run(["curl", "-o", assets_env_path + git_file_path, full_url], check=True)
    # print(result.stdout)

if __name__ == "__main__":
    # h_assets_exist("/root/h12_sim_assets")
    bootstrap_assets()









