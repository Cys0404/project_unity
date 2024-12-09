# Example scene of connect to real arm
Here are 4 different example scenes for learning how to connect to the real arm, to get the real arm status, to control the arm.  
1. The connection technique is base on ROS# to connect the ROS2 Rosbridge server. Please check your ROS# version is later than 2.0.0 for supporting ROS2 communication.
2. check [koch_ros2_wrapper](https://github.com/hrc-pme/koch_ros2_wrapper) for real reobot setup and run the real robot.
## Important components
Here are some important components in these example project - XXX_connector_OOO.
It is the connector gameobject (An empty gameobject + ros connector scripts), check some important pararmeter.
* Ros Connector
- Ros Bridge Server Url: the IP and port to connect to robot. (format: ws://{IP}:{port} e.g. ws://192.168.0.244:9090)
- ROS Version: connect to what ros version (e.g. ROS1, ROS2) 
* Publisher / Subscriber
- Topic: the topic will subscribe or publish to
Important: Carefully check the name or ip you type in, any indent or space will make the system takes it as a different name. 

## Example 1: Control_from_leader_example
In this example, the arm in unity like a simulator, the user need to run the real leader arm and use the leader arm to control the follower in the unity.
* For real arm, check [single_leader_control](https://github.com/hrc-pme/koch_ros2_wrapper#setup-1-single-leader) to run single leader control.
* For unity, check the IP, topic name setup is correct.

## Example 2: Status_from_follower_example
In this example, the unity get the joint_states data from the real follower arm, like a virtual represent arm in unity.
* For real arm, check [leader_follower_control](https://github.com/hrc-pme/koch_ros2_wrapper#setup3-single-leader-follower) 
* For unity, check the IP, topic name setup is correct.

## Example 3: keyboard_control_example
In this example, we will have two arm in unity: control arm and real arm
the A/Z S/X D/C F/V G/B H/N on keyboard can control joint1 ~ 6, and "Space" will send the joint_state data to real arm to control the arm. You can press one time to do goal point control, or keep pressing for dragging control.
* For real arm, check [single_follower_control](https://github.com/hrc-pme/koch_ros2_wrapper#setup2-single-follower)
* For unity, check the IP and topic name setting is correct. 
## Example 4: leader_follower_example
In this example, the arm in unity get the leader arm joint_state and send to the follower arm. (compare to the single_leader_follower control, this method can seperate leader and follower arm to two different machine.)
Control flow chart:
single_leader_follower: leader -> follower (need on same computer)
leader_follower_example: leader -> unity -> follower (leader follower can seperate to two different machine or still on the same.)
* For real arm, check [leader_follower_control](https://github.com/hrc-pme/koch_ros2_wrapper#setup3-single-leader-follower) 
* For unity, check the IP, topic name setup is correct.
 