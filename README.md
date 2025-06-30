# Bi-ManualManipulation

```
sudo apt install libyaml-cpp-dev
```

<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>opencv</depend>

vcs import for allegro_hand_description kinova gen3

git clone -b ros2 https://gitioc.upc.edu/hands/allegro_hand_ros.git
mv allegro_hand_ros/allegro_hand_description/ .
rm -rf allegro_hand_ros/

in `allegro_hand_description/urdf/rightAllegroHand.urdf.xacro`
change 
<mesh filename="package://allegro_hand_description/meshes/link_3.0_tip.STL"/>
to 
<mesh filename="package://allegro_hand_description/meshes/link_3_tip.STL"/>

git clone https://github.com/Kinovarobotics/ros2_kortex.git -b main
mv ros2_kortex/kortex_description/ .
rm -rf ros2_kortex/

sudo apt install ros-humble-kdl-parser-py

Adapting kdl_parser_py to ROS 2

cd ~/FuturHand/src/kdl_parser/kdl_parser_py
rm CMakeLists.txt AMENT_IGNORE

setup.py
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

package.xml
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

setup.cfg
```
[develop]
script_dir=$base/lib/kdl_parser_py
[install]
install_scripts=$base/lib/kdl_parser_py
```

file structure
```
kdl_parser_py/
  ├── kdl_parser_py/
  │   ├── __init__.py
  │   └── urdf.py
  ├── setup.py
  ├── setup.cfg
  └── package.xml
```

Replace:

kdl.Joint(j.name, kdl.Joint.None)

with:

kdl.Joint(j.name, kdl.Joint.Fixed)

limitations:
note: bimanual_sensor currently only uses the right camera. it also needs better params management in launch
note: left hand given grasp priority duirng handoff in the simulator, couldn't think of justifiable logic for this so defaulted to task-based priority