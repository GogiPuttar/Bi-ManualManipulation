# Bi-ManualManipulation

# Overview

# System Design

# Packages

## `bimanual_mockup` package

## `bimanual_sensor` package

## `bimanual_controller` package

## `bimanual_planner` package

## `bimanual_msgs` package

# Setup and Instructions

## REQUIREMENTS:

**TODO:** `vcs import` for dependencies
- ROS 2 Humble (This code was tested on Ubuntu 22.04+)
- ROS 2 TF2 Transformations 
  ```
  sudo apt install ros-humble-tf*
  ```
- C++ YAML Library
  ```
  sudo apt install libyaml-cpp-dev
  ```
- ROS 2 CV Bridge
  ```
  sudo apt install ros-humble-cv-bridge
  ```
- ROS 2 Image Transport
  ```
  sudo apt install ros-humble-image-transport
  ```
- Open CV (C++ and Python)
  ```
  sudo apt-get install python3-opencv 
  pip install opencv-python
  ```
- Allegro Hand for ROS 2 (`allegro_hand_description` package)
  In `src`:
  ```
  git clone -b ros2 https://gitioc.upc.edu/hands/allegro_hand_ros.git
  mv allegro_hand_ros/allegro_hand_description/ .
  rm -rf allegro_hand_ros/
  ```
  **NOTE:** Broken URDF. In `allegro_hand_description/urdf/rightAllegroHand.urdf.xacro`
  change: 
  ```
  <mesh filename="package://allegro_hand_description/meshes/link_3.0_tip.STL"/>
  ```
  to: 
  ```
  <mesh filename="package://allegro_hand_description/meshes/link_3_tip.STL"/>
  ```
- Kinova Robot Arm for ROS 2 (`kortex_description` package)
  In `src`:
  ```
  git clone https://github.com/Kinovarobotics/ros2_kortex.git -b main
  mv ros2_kortex/kortex_description/ .
  rm -rf ros2_kortex/
  ```
- Kinematics and Dynamics Library (KDL) Parser (`kdl_parser` and `kdl_parser_py` package)
  ```
  sudo apt install ros-humble-kdl-parser-py
  ```
  In `src`:
  ```
  git clone git@github.com:ros/kdl_parser.git -b humble
  ```
  **NOTE:** Broken package `kdl_parser_py` meant for ROS 1. Adapt this to ROS 2 by:
  ```
  cd ~/FuturHand/src/kdl_parser/kdl_parser_py
  rm CMakeLists.txt AMENT_IGNORE
  ```
  Change `setup.py` to:
  ```
  from setuptools import setup

  package_name = 'kdl_parser_py'

  setup(
      name=package_name,
      version='2.6.4',
      packages=[package_name],
      install_requires=['setuptools'],
      zip_safe=True,
      maintainer='Your Name',
      maintainer_email='your@email.com',
      description='Python parser for KDL trees from URDF models',
      license='BSD',
      tests_require=['pytest'],
      entry_points={},
  )
  ```
  Change `package.xml` to:
  ```
  <?xml version="1.0"?>
  <package format="3">
    <name>kdl_parser_py</name>
    <version>2.6.4</version>
    <description>Python tools to construct a KDL tree from a URDF model.</description>

    <maintainer email="you@yourdomain.com">Your Name</maintainer>
    <license>BSD</license>

    <buildtool_depend>ament_python</buildtool_depend>
    <exec_depend>urdfdom_py</exec_depend>
    <exec_depend>python_orocos_kdl_vendor</exec_depend>

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
      <build_type>ament_python</build_type>
    </export>
  </package>
  ```
  Add `setup.cfg`:
  ```
  [develop]
  script_dir=$base/lib/kdl_parser_py
  [install]
  install_scripts=$base/lib/kdl_parser_py
  ```
  Final skeleton should look like:
  ```
  kdl_parser_py/
    ├── kdl_parser_py/
    │   ├── __init__.py
    │   └── urdf.py
    ├── setup.py
    ├── setup.cfg
    └── package.xml
  ```
  Finally, in `urdf.py` replace:
  ```
  kdl.Joint(j.name, kdl.Joint.None)
  ```
  with:
  ```
  kdl.Joint(j.name, kdl.Joint.Fixed)
  ```

# Limitations
note: bimanual_sensor currently only uses the right camera. it also needs better params management in launch
note: left hand given grasp priority duirng handoff in the simulator, couldn't think of justifiable logic for this so defaulted to task-based priority

# Short Term Improvements

side
https://github.com/user-attachments/assets/e9db8ec5-ab15-402c-b95b-d46f3bd75851

front
https://github.com/user-attachments/assets/e010c9ab-8f5f-493a-a890-85126b13ac96

topfront
https://github.com/user-attachments/assets/84ca44f5-4265-426b-a53f-60167f12d50e

grasp
https://github.com/user-attachments/assets/7d593644-72a3-4f06-a365-11dd326f88a1


