# How to run the code

To run the code the following things are needed:
- ROS noetic
- *package assignment_2_2024*
After cloning the nedded packages in the workspace and after sourcing the right setup files, the workspace has to be compiled with *catkin make*.

Then to run the nodes you have to go in the workspace folder and run the following command:

		 - roslaunch assignment_2_part1 assign2_p1_sim.launch
The other relevant commands are the followings:

		 - rostopic echo /robot_state

To listen to the topic where the custom message is sent.
And the last one is:

	- rosservice call last_target_coord

to call the custom service.


# What I have done
I have built a ROS package  called *assignment_2_part1*, that contains the following things:
- In the folder *src*: two .cpp files that correspond to the two nodes of the package. They are *action_server_client.cpp* and *last_target_srv.cpp*.
- In the folder *srv*: *target_coord.srv*, a custom service.
- In the folder *launch*: *assign2_p1_sim.launch*, a file to launch the whole simulation.
- In the folder *msg*: *state_pos_vel.msg*, a custom message.

## Node 1: action_server_client.cpp

This node implements an action client for the action server given in the package *assignment_2_2024*.
The node that does the following things:
- allows the user to set a target, using x and y coordinates;
- allows the user to cancel the target, stopping the robot. If this happens, the message "The current goal has been canceled. The robot will stop." is written on the terminal;
- allows the user to close the node;
- permits to know when the target is reached, by reading the feedback message sent by the action server. If this happens, the message "The target has been reached!" is written on the terminal;
- publishes the robot position and velocity with a custom message (containing x, y, vel_x, vel_z). These informations are retrieved by a subscriber to the topic */odom*. To handle the callback of the subscriber, the nodes launches a dedicated thread (for the ros::spin() instruction). 

## Node 2: last_target_srv.cpp

This one is a service node and its objective is to return the coordinates of the last target set by the user, reading them from the topic */target_coordinates*.
This topic is updated by the previous node each time that a new target is set.

When the service is called its callback function reads the topic and returns the last values. 
