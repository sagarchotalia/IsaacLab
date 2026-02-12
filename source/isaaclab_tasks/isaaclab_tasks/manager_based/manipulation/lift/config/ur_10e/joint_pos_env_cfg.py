# Copyright (c) 2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from isaaclab.assets import ArticulationCfg, RigidObjectCfg, AssetBaseCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg, OffsetCfg
from isaaclab.utils import configclass
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.managers import SceneEntityCfg
import isaaclab.sim as sim_utils
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg
from isaaclab_assets.robots.universal_robots import UR10e_ROBOTIQ_GRIPPER_CFG, UR7e_with_RG6_Gripper_CFG  # isort: skip
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm

##
# Environment configuration
##


@configclass
class UR10eLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.robot = UR10e_ROBOTIQ_GRIPPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        #self.events.joint_friction.params["asset_cfg"].joint_names = ["shoulder_.*", "elbow_.*", "wrist_.*"]

        # Override the reset object position event to use the correct body name "Beaker"
        #self.events.reset_object_position.params["asset_cfg"] = SceneEntityCfg("object", body_names="ErlenmeyerFlask_01")

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=[
                "shoulder_.*","elbow_joint","wrist_.*"
            ],
            scale=0.5,
            use_default_offset=True
        )
        # robotiq gripper
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger_joint"],
            open_command_expr={"finger_joint": math.radians(-30.0)},
            close_command_expr={"finger_joint": math.radians(30.10)},
        )
        # self.commands.object_pose.body_name = "onrobot_rg6_base_link" # originally robotiq_arg2f_base_link # UR7e
        self.commands.object_pose.body_name = "robotiq_arg2f_base_link"
        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
        # reducing the object in air reward to solve the issue of throwing cube around
        
        # self.rewards.lifting_object.weight = 5.0
        # self.rewards.reaching_object.weight = 3.0
        # self.rewards.gripper_close = RewTerm(
        #     func=mdp.gripper_closed_object_picked,
        #     weight=3.0,
        # )
        # self.rewards.object_drop = RewTerm(
        #     func=mdp.object_dropped_penalty,
        #     weight=-5.0,
        #     params={"minimal_height":0.04}
        # )
        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/world",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/ee_link/robotiq_arg2f_base_link",
                    # prim_path="{ENV_REGEX_NS}/Robot/onrobot_rg6_model/onrobot_rg6_base_link" # for UR7e
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],
                    ),
                ),
            ],
        )

@configclass
class UR10eLiftEnvCfg_PLAY(UR10eLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False


# Add this once the policy is trained with the cube.
# Things to try:
#   - Change the scale (issue of scale with the silhouette being tiny but the collision meshes being the perfect size)
#   - Change the units of measurement of the LabUtopia assets into metres. it's possible they are currently in cm or inches
# self.scene.object = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Object",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.15], rot=[1, 0, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/home/spark/Desktop/embodied_ai_repository/Assets/ErlenmeyerFlask.usd",
        #         scale=(1.0, 1.0, 1.0),
        #         rigid_props=RigidBodyPropertiesCfg(
        #             solver_position_iteration_count=16,
        #             solver_velocity_iteration_count=1,
        #             max_angular_velocity=1000.0,
        #             max_linear_velocity=1000.0,
        #             max_depenetration_velocity=5.0,
        #             disable_gravity=False,
        #         ),
        #     ),
        # )
