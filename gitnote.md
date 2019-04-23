
- what nodes are running after `roslaunch panda_moveit_config demo.launch`?

```
/joint_state_desired_publisher   //?????

/joint_state_publisher
/robot_state_publisher
/virtual_joint_broadcaster_1

/move_group

/rviz_ibm_28714_7890803671492789067    # "ibm" is the host name
/rosout
```
- [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)
- [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
- joint_state_desired_publisher
    - Franka_ros 相关的节点。
    - ASUS 没有这个节点，一切运行正常
- After running 'roslaunch panda_moveit_config joystick_control.launch'
```
/joint_state_desired_publisher
```

