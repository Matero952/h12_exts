# from omni.isaac.core.utils.stage import add_reference_to_stage
# # from isaaclab.sim.converters import *
import isaaclab.sim as sim_utils

# import asyncio
# import isaaclab.sim as sim_utils

def spawn_asset():
    prim = sim_utils.spawn_from_usd(
        prim_path="/World/envs/env_0/robot_test",
        cfg=sim_utils.UsdFileCfg(usd_path="/root/unitree_model/H1-2/h1_2/h1_2.usd")
    )
    return prim

# Schedule it to run
# asyncio.ensure_future(spawn_asset())

