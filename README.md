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

git clone https://github.com/Kinovarobotics/ros2_kortex.git -b main
mv ros2_kortex/kortex_description/ .
rm -rf ros2_kortex/

limitations:
note: bimanual_sensor currently only uses the right camera. it also needs better params management in launch