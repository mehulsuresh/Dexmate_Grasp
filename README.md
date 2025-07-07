# Dexmate Lift Task Integration for IsaacLab

This repository contains the `dexmate_lift` task, designed to be integrated with the IsaacLab simulation platform. Follow the instructions below to set up and run this task within your IsaacLab environment.

## Prerequisites

Before proceeding, ensure you have a working installation of IsaacLab. If not, please refer to the official  documentation for installation instructions to install Isaacsim and Isaaclab: [pip installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html).

## Setup Instructions

### Step 1: Update the source\isaaclab\isaaclab\envs\manager_based_rl_env.py file

To ensure proper functionality of certain reward functions, a specific modification is required within the `manager_based_rl_env.py` file in your IsaacLab installation.

You need to add the following two lines inside the `_reset_idx` function within the `IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env.py` file. This initializes the `init_quat_w` attribute for the `object` with its root quaternion upon each environment reset.

Locate the `_reset_idx` function and insert the lines as shown below:

```python

class ManagerBasedRLEnv(RLBaseEnv):
    # ... (existing methods) ...

    def _reset_idx(self, env_ids: Sequence[int]):
        """Resets the environment for the given environment IDs.

        Args:
            env_ids: The environment IDs to reset.
        """
        # existing code
        # --- START OF REQUIRED ADDITION ---
        # Initialize object's initial quaternion for reward calculation
        object = self.scene["object"]
        object.data.init_quat_w = object.data.root_quat_w.clone()
        # --- END OF REQUIRED ADDITION ---
```

### Step 2: Move the dexmate.py file to the assets folder:

`source\isaaclab_assets\isaaclab_assets\robots\dexmate.py`

### Step 3: Move the USD folder in this repository to the base isaaclab repository

### Step 4: Place the dexmate_lift Repository

Navigate to the source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation directory within your cloned IsaacLab repository. This is where the dexmate_lift task should reside.

### To run training:
isaaclab.bat -p scripts/reinforcement_learning/rl_games/train.py --task=Isaac-Lift-Cube-Dexmate-v0 --headless   

### To run inference:
 isaaclab.bat -p scripts/reinforcement_learning/rl_games/play.py --task=Isaac-Lift-Cube-Dexmate-v0 --num_envs=8 --checkpoint "checkpoint_file_path.pth"   