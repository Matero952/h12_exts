from dataclasses import dataclass
from typing import Union
from pxr import Usd, Sdf
import omni.usd
from isaacsim.sensors.camera import Camera

# from p
@dataclass
class SimCameras():
    """Class to list sim camera prims"""
    stage: Union[Usd.Stage, None] = omni.usd.get_context().get_stage()
    sim_cameras_list: Union[list[Camera], None] = None

    def set_sim_cameras_list(self):
        sim_camera_list = []
        if not self.stage:
            return None
        else:
            for prim in self.stage.Traverse():
                if not prim.GetPrimPath().HasPrefix("/World/envs/env_0/Robot"):
                    #all 'Camera' type prims deeper than this prefix are what we want
                    continue
                if (prim.GetPrimTypeInfo().GetTypeName().strip() == "Camera"):
                    sim_camera_list.append(Camera(str(prim.GetPrimPath())))
            self.sim_cameras_list = sim_camera_list
            return None
        
class SimCamerasManager(SimCameras):
    def __init__(self):
        super().__init__()
        self.set_sim_cameras_list()
        print(self.sim_cameras_list)
        #set 'Camera' objects

    def get_camera_info()

    
