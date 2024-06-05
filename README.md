[![DHSG](https://img.shields.io/badge/KTH-DHSG-green)](https://www.example.com/dhsg) [![ROS2 Humble Version](https://img.shields.io/badge/ROS2-Humble-orange)](https://www.example.com/ros2)






# Nexus Robot move base plugin for Gazebo :robot:

This package contains a ROS2 plugin for Gazebo Classic 11 to move the holonomic nexus platforms according to a given `cmd_vel` commands. 

## The nexus platform
Nexus is an holonomic robot platform that is commonly applied for robotics research. You can find many different nexus models [here](https://www.nexusrobot.com/product/product-category/robot-kits). Every robot has different dynamics and applications. At KTH, the [mecanum wheel model](https://www.nexusrobot.com/product/4wd-mecanum-wheel-mobile-arduino-robotics-car-10011.html) is the one we commonly use



![Alt text](resources/nexus.png?raw=true "Title")

## What is this package for ?
This package contains the gazebo plugin `sml_nexus_ros2_force_based_move.cpp` to move the nexus robot in the [Gazebo Simulator](https://classic.gazebosim.org/). A plugin is a chunk of `cpp` that can be used to directly interact with the simulator and many plugins are already available for many different purposes like simulating sensors, controllers and so on. 

To understand how to code your own plugin, it is strongly recommended to have a look at examples from the ros developers here : 

https://github.com/ros2-gbp/gazebo_ros_pkgs-release/blob/release/humble/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp

One important fact about Gazebo that it is often misunderstood is that Gazebo is not developed for ROS. Hence, gazebo plugins developed by the ROS developers typically look a bit different than the ones developed by the gazebo developer. The main difference being that gazebo plugin for ros typically contain a ros node inside so that messages from the simulator to various ros topics can be sent directly from the plugin. This is the case for the plugin in this repo as you can see by inspecting the code yourself at the line.

```
// Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
```

## How to spawn the nexus in gazebo and control them ?
This package only contain the plugin to move the nexus robots but it doe not have any description file. To spawn the nexus and control them we refer to the nexus description package [here](https://github.com/gregoriomarchesini/sml_nexus_description_ros2). 


## Requirements
The package was tested for ROS2 Humble. An installation of Gazebo versions 9-11 is required to launch the simulator. Gazebo 11 is the currently tested version.


## Installation 

If you haven't done so, create a ROS2 workspace 

```
mkdir -p nexus_ws/src
cd nexus_ws/src
```
from the `src` folder install the following repos

```
git clone https://github.com/gregoriomarchesini/sml_nexus_description_ros2
git clone https://github.com/gregoriomarchesini/sml_nexus_description_ros2.git
git clone https://github.com/gregoriomarchesini/sml_nexus_tutorials_ros2.git
```

Then build the workspace 

```
cd ..
colcon build --symlink-install
```
You are now ready to run your nexus

```
ros2 launch sml_nexus_tutorials_ros2 sml_nexus_mas_gazebo.launch.py
```

You will see three robots 


Once you spawned your robot you can publish to the `cmd_vel` topic for each agent.

```
ros2 topic pub agent1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}' -r 20
```


Happy coding! :smile:


## License

This package is distributed under the [Apache License 2.0](LICENSE).

## Authors

Gregorio Marchesini [gremar@kth.se](mailto:gremar@kth.se).
