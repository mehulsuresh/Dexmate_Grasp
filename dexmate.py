# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause



import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

DEXMATE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        
        usd_path=f"usd\dexmate.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "R_arm_j1": 0.0,
            "head_j1": 0.0,
            "R_arm_j2": 0.0,
            "head_j2": 0.0,
            "R_arm_j3": 0.0,
            "head_j3": 0.0,
            "R_arm_j4": 0.0,
            "R_arm_j5": 0.0,
            "R_arm_j6": 0.0,
            "R_arm_j7": 0.0,
            "RR_ff_j1": 0.0,
            "RR_lf_j1": 0.0,
            "RR_mf_j1": 0.0,
            "RR_rf_j1": 0.0,
            "RR_th_j0": 0.0,
            "RR_ff_j2": 0.0,
            "RR_lf_j2": 0.0,
            "RR_mf_j2": 0.0,
            "RR_rf_j2": 0.0,
            "RR_th_j1": 0.0,
            "RR_th_j2": 0.0,
    },
    ),    
    actuators={
        "panda_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["R_arm_j.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=10.0,
            damping=1.0,
        ),
        "panda_hand": ImplicitActuatorCfg(
            joint_names_expr=["RR.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=50.0,
            damping=1.0,
        ),
        "panda_head": ImplicitActuatorCfg(
            joint_names_expr=[ "head_j.*"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=10.0,
            damping=1.0,
        ),
    # ['R_arm_j1', 'head_j1', 'R_arm_j2', 'head_j2', 'R_arm_j3', 'head_j3', 'R_arm_j4', 'R_arm_j5', 'R_arm_j6', 'R_arm_j7', 'R_ff_j1', 'R_lf_j1', 'R_mf_j1', 'R_rf_j1', 'R_th_j0', 'R_ff_j2', 'R_lf_j2', 'R_mf_j2', 'R_rf_j2', 'R_th_j1', 'R_th_j2']
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Franka Emika Panda robot."""


DEXMATE_HIGH_PD_CFG = DEXMATE_CFG.copy()
DEXMATE_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
"""Configuration of Franka Emika Panda robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
