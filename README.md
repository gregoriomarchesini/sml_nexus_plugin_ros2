[![DHSG](https://img.shields.io/badge/KTH-DHSG-green)](https://www.example.com/dhsg) [![ROS2 Humble Version](https://img.shields.io/badge/ROS2-Humble-orange)](https://www.example.com/ros2)
# Nexus Robot move base plugin for Gazebo :robot:
This package contains a ROS2 plugin for Gazebo Classic 11 thst can be applied to move the nexus platforms according to a given `cmd_vel` command. You can spawn your robot using [this package](https://github.com/gregoriomarchesini/sml_nexus_description_ros2). 

```
git clone https://github.com/gregoriomarchesini/sml_nexus_description_ros2
```
Once you spawned your robot you can publish to the following `cmd_vel` topic.
```
ros2 topic pub nexus/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2, y: 0.0, z: 0.0}, angular: {x: 1.0, y: 0.0, z: 0.0}}' -r 20
```


Happy coding! :smile:


## License

This package is distributed under the [Apache License 2.0](LICENSE).

## Authors

Gregorio Marchesini [gremar@kth.se](mailto:gremar@kth.se).
