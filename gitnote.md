# Inside `roslaunch panda_moveit_config demo.launch`
```
   - <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
   - <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
   - <node name="virtual_joint_broadcaster_0" pkg="tf" type="static_transform_publisher"> 
      - or <node name="virtual_joint_broadcaster_1" pkg="tf2_ros" type="static_transform_publisher" >
   <node name="joint_state_desired_publisher" pkg="topic_tools" type="relay" >
```
- Invoved Package/Node
   - [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
           /joint_state_publisher
   - [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)
           /robot_state_publisher
   - [tf2_ros](http://wiki.ros.org/tf2_ros) 
           /virtual_joint_broadcaster_1
      - [related](http://wiki.ros.org/tf2) 
   - [topic_tools](http://wiki.ros.org/topic_tools)
           /joint_state_desired_publisher  # This is not a ROS official node!
- Involved Msg/Topic
   - Topic::[sensor_msgs](http://docs.ros.org/api/sensor_msgs/html/index-msg.html)
            /[JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
- what nodes are running after 
```
/joint_state_publisher
/robot_state_publisher
/virtual_joint_broadcaster_1
/joint_state_desired_publisher

/move_group

/rviz_ibm_28714_7890803671492789067    # "ibm" is the host name
/rosout
```

- After running 'roslaunch panda_moveit_config joystick_control.launch'
```
/joint_state_desired_publisher
```

