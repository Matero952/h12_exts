import asyncio
import gc
import sys
import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType
import carb
print(sys.path)
sys.path.append("/root/h12_exts/source/extensions/h12_assets/h12_assets_python")
print(sys.path)

# from load_assets_in_stage import *

from .global_variables import EXTENSION_DESCRIPTION, EXTENSION_TITLE
from .ui import UIBuilder

"""
This file serves as a basic template for the standard boilerplate operations
that make a UI-based extension appear on the toolbar.

This implementation is meant to cover most use-cases without modification.
Various callbacks are hooked up to a seperate class UIBuilder in .ui_builder.py
Most users will be able to make their desired UI extension by interacting solely with
UIBuilder.

This class sets up standard useful callback functions in UIBuilder:
    on_menu_callback: Called when extension is opened
    on_timeline_event: Called when timeline is stopped, paused, or played
    on_physics_step: Called on every physics step
    on_stage_event: Called when stage is opened or closed
    cleanup: Called when resources such as physics subscriptions should be cleaned up
    build_ui: User function that creates the UI they want.
"""


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        self.ui_builder = UIBuilder()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)

        # Filled in with User Functions
        

        # Events
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.get_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()
        print(f"H12 EXTENSION STARTED UP")
        carb_settings = carb.settings.get_settings()
        assert carb_settings is not None
        # carb.settings.get_settings().set_int("/rtx/debugMaterialType", 0)
        # # carb.settings.get_settings().set_int()
        # # carb_settings = carb.settings.get_settings()
        # # assert carb_settings is not None
        # #set execMode to performance so that shit doesnt fucking break my computer:
        # carb_settings.set("/rtx/post/dlss/execMode", 0)
        # #should boost performance:
        # carb_settings.set("/rtx/post/aa/op", 3)
        # #dont even think this can run on my computer but just set it to false anyways:
        # carb_settings.set("/rtx-transient/dlssg/enabled", False)
        # # carb_settings.set("/rtx/raytracing/showLights", 2)
        # carb_settings.set_int("/rtx/debugMaterialType", 0)
        # from omni.isaac.core.utils.stage import add_reference_to_stage

        # import carb 
        
        carb_settings.set("/log/debugConsoleLevel", "Debug")  # verbose"|"info"|"warning"|"error"|"fatal"
        carb_settings.set("/log/enabled", False)
        carb_settings.set("/log/outputStreamLevel", "Debug")
        carb_settings.set("/log/fileLogLevel", "Debug")
        # # from isaaclab.sim.converters import *
        # from load_assets_in_stage import spawn_asset
        # spawn_asset()
        # import isaacsim.core.utils.stage as stage_utils
        # stage_utils.add_reference_to_stage(usd_path="/root/h12_sim_assets/assets/chainsaw/mini_chainsaw_scan_lowpoly.usd", prim_path="/World/mini_chainsaw_scan_lowpoly")
        # from ui import _get_object_names
        # _get_object_names()
        # import isaaclab.sim as sim_utils
        # from pxr import Usd, UsdPhysics, UsdGeom
        # stage = Usd.Stage.Open("/root/h12_sim_assets/assets/drill/low-poly-drill.usdc")
        # root_prim = stage.GetDefaultPrim()
        # if not root_prim:
        #     print(stage)
        # print(root_prim)
        # breakpoint()
        # UsdPhysics.RigidBodyAPI.Apply(root_prim)
        # UsdPhysics.CollisionAPI.Apply(root_prim)
        # UsdPhysics.MassAPI.Apply(root_prim)
        # UsdPhysics.ArticulationRootAPI.Apply(root_prim)
        # # rootd_prim.SetInstanceable(True)
        # stage.GetRootLayer().Save()
        # print(f"Ok i did that shit")
        #     # root_prim = stage.GetPrimAtPath()
        # prim = sim_utils.spawn_from_usd(
        # prim_path="/World/t22est",
        # cfg=sim_utils.UsdFileCfg(usd_path="/root/h12_sim_assets/assets/wrench/adjustable_wrench.usd")
        # )
        # print("tried to add")

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        print("H12 EXTENSION SHUTDOWN")
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        def dock_window():
            omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        self.ui_builder.on_physics_step(step)
        pass

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        # Call user function for building UI
        self.ui_builder.build_ui()
        # self.ui_builder.object_dropdown.repopulate()
        pass
