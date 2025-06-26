# Bi-ManualManipulation

# Requirements:
1. Ubuntu 20.04+
   - This package was tested on Ubuntu 22.04.
2. ROS 2 Humble. ([Install guide](https://docs.ros.org/en/humble/Installation.html)) 
3. NVIDIA Isaac Sim. ([Download](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html))
   - This package was built using version [4.5.0-rc.36](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.linux-x86_64.release.zip).

# Set up and Instructions
(As of June 26th, 2025)

THIS SECTION IS INCOMPLETE. DEVELOPMENT ON THIS PACKAGE IS UNDER PROGRESS.

1. Add this to your `.bashrc` (aliasing these commands is optional, but they still need to be run!):
    ```
    ### FuturHand Bimanual task setup ###

    # Define environment-specific variables
    export ISAAC_SIM_DIR=~/Downloads/isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.linux-x86_64.release # Replace with your Isaac Sim directory
    export FUTURHAND_WS=~/FuturHand # Replace with the ROS 2 workspace
    export BIMANUAL_REPO=~/FuturHand/src/Bi-ManualManipulation  # Replace with the path into this cloned repo

    # 
    alias humble='
    unset CMAKE_PREFIX_PATH
    unset AMENT_PREFIX_PATH
    source /opt/ros/humble/setup.bash
    echo $CMAKE_PREFIX_PATH
    echo $AMENT_PREFIX_PATH
    echo $ROS_DISTRO
    '

    # Isaac Sim + ROS 2 workspace setup alias
    alias isaacsetup='
    export ISAAC_PATH=$ISAAC_SIM_DIR
    export CARB_APP_PATH=$ISAAC_SIM_DIR
    export EXP_PATH=$BIMANUAL_REPO/bimanual_isaac/assets

    # ROS 2 Humble setup (adjust as needed)
    humble

    # Source local ROS 2 workspace (adjust as needed)
    source $FUTURHAND_WS/install/setup.bash

    # Change to Isaac Sim directory
    cd $ISAAC_SIM_DIR
    '

    # Run Isaac Sim alias
    alias isaacrun ='
    # TODO
    '
    ### End of FuturHand Bimanual task setup ###
    ```