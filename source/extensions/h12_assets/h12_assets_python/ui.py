import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import create_new_stage, get_current_stage
# from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux
from utils import spawn_asset, get_h12_assets_path, get_dir_usd_file
import os
import omni.usd
from isaacsim.gui.components import DropDown, StringField, TextBlock

import omni.kit.commands


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()
        self.stage = omni.usd.get_context().get_stage()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
            # For complete robustness, the user should resolve those edge cases here
            # In general, for extensions based off this template, there is no value to having the user click the play/stop
            # button instead of using the Load/Reset/Run buttons provided.
            # self._scenario_state_btn.reset()
            pass
            # self._scenario_state_btn.enabled = False
        pass
    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        # world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        # with world_controls_frame:
        #     with ui.VStack(style=get_style(), spacing=5, height=0):
        #         self._load_btn = LoadButton(
        #             "Load Button", "LOAD", setup_scene_fn=self._setup_scene, setup_post_load_fn=self._setup_scenario
        #         )
        #         self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
        #         self.wrapped_ui_elements.append(self._load_btn)

        #         self._reset_btn = ResetButton(
        #             "Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
        #         )
        #         self._reset_btn.enabled = False
        #         self.wrapped_ui_elements.append(self._reset_btn)

        dropdown_frame = CollapsableFrame("Assets")

        with dropdown_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # self.object_dropdown = DropDown(
                #     label = "Objects",
                #     tooltip = "H12 Objects Spawn and Despawn",
                #     populate_fn = self._get_object_names,
                #     on_selection_fn = self.spawn_or_despawn_object
                # )
                self._enable_disable_object_stringfield = StringField(
                    label = "enable/disable stringfield",
                    tooltip = f"type 'enable' and the object name that you want to enable. Possible object names rn are: {self._get_object_names()}",
                    default_value = '',
                    read_only = False,
                    multiline_okay = False,
                    on_value_changed_fn = self.parse_and_execute_input_spawn_or_despawn_stringfield_val
                )
                self.guide_text_block = TextBlock(
                    label = "Hint",
                    text = f"type 'enable' or 'disable' and the object name that you want to enable/disable. Possible object names rn are: {self._get_object_names()}. For example, to spawn the drill object, type: 'enable drill'",
                )
                # for i in self._get_object_names():
                #     DropDown(
                #         label = i,
                #         tooltip = f"{i} spawn/despawn",
                #         populate_fn = self._get_object_names,

                #     )

                    # Button(i, f"{i} object spawn/despawn", on_click_fn=self.spawn_or_despawn_object)
                    
        # await self._object_dropdown.repopulate()


    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################
    def _dropdown_on_selection(self, object: str):
        import json
        asset_manifest_path = self._get_asset_manifest_path()
        with open(asset_manifest_path, 'r') as f:
            data = json.load(f)
        spawn_asset(usd_path=data['assets'][object]['usdz_file'])
        
        

    def _on_init(self):
        self._articulation = None
        self._cuboid = None

    def _add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

    def _setup_scene(self):
        """
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.
        """
        create_new_stage()
        self._add_light_to_stage()

        # loaded_objects = self._scenario.load_example_assets()

        # Add user-loaded objects to the World
        world = World.instance()
        for loaded_object in loaded_objects:
            world.scene.add(loaded_object)

    def _setup_scenario(self):
        """
        This function is attached to the Load Button as the setup_post_load_fn callback.
        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.
        """
        # self._scenario.setup()

        # UI management
        # self._scenario_state_btn.reset()
        # self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        # self._scenario.reset()

        # UI management
        # self._scenario_state_btn.reset()
        # self._scenario_state_btn.enabled = True
        pass

    def _update_scenario(self, step: float):
        """This function is attached to the Run Scenario StateButton.
        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        This function will repeatedly advance the script in scenario.py until it is finished.

        Args:
            step (float): The dt of the current physics step
        """
        # done = self._scenario.update(step)
        # if done:
            # self._scenario_state_btn.enabled = False
        pass

    def _on_run_scenario_a_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_a_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "RUN".

        This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
        the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
        through the left-hand UI toolbar.
        """
        self._timeline.play()

    def _on_run_scenario_b_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_b_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "STOP"

        Pausing the timeline on b_text is not strictly necessary for this example to run.
        Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
        the robot will stop getting new commands and the cube will stop updating without needing to
        pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
        forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
        this example prettier, but if curious, the user should observe what happens when this line is removed.
        """
        self._timeline.pause()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        # self._scenario_state_btn.reset()
        # self._scenario_state_btn.enabled = False
        # self._reset_btn.enabled = False
        pass

    def parse_and_execute_input_spawn_or_despawn_stringfield_val(self, stringfield: str):
        words = stringfield.split(' ')
        for idx, i in enumerate(words):
            words[idx] = i.strip()
        if words[0] not in ["enable", "disable"] or words[1] not in self._get_object_names():
            return
        if words[0] == "enable":
            if self.stage.GetPrimAtPath(f"/World/h12_assets/{words[1]}").IsValid():
                return
            spawn_asset(prim_path=f"/World/h12_assets/{words[1]}", usd_path=get_dir_usd_file(get_h12_assets_path() + "/assets/" + words[1]))
        elif words[0] == "disable":
            omni.kit.commands.execute("IsaacSimDestroyPrim",prim_path=f"/World/h12_assets/{words[1]}")
            print(words[1])

    def _get_object_names(self):
        return [path for path in os.listdir(get_h12_assets_path() + "/assets")]



        
    
