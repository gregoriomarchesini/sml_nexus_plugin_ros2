You can control your robot by publishing over the topic `nexus\cmd_vel`

```
ros2 topic pub nexus/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2, y: 0.0, z: 0.0}, angular: {x: 1.0, y: 0.0, z: 0.0}}' -r 20
```

You can retrieve the pose of your model from gazebo using the mocap topic. For example :
```
ros2 topic echo /mocap_simulator/model_states_mocap
```
gives the output

```
name:
- ground_plane
- sml_nexus
pose:
- position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: 0.002907976184165049
    y: 0.019097476655150634
    z: -5.215579993597674e-06
  orientation:
    x: -1.2659576823572743e-05
    y: -4.090912071999571e-06
    z: 2.099972629919146e-05
    w: 0.9999999996910056
twist:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
- linear:
    x: 0.0009170311313763605
    y: -0.0017961573714265844
    z: -0.005537430128307253
  angular:
    x: -0.03542707982748293
    y: -0.018053289657945292
    z: 1.7648212936216685e-05
---

```

sensor reading can be obtained in a similar way.

Happy coding!


## License

This package is distributed under the [Apache License 2.0](LICENSE).

## Authors

Gregorio Marchesini 
[gremar@kth.se](mailto:gremar@kth.se).
