/**
* \file action_server_client.cpp
* \brief This node implements an action client for the action server in the package 'assignment_2_2024'.
* \author Barbara Ratto
* \version 1.0
* \date 12/03/25
*
* \details
*
* **Subscribes to**: <BR>
* - /odom
*
* **Publishes to**: <BR>
* - /robot_state
* - /target_coordinates
*	
* **Action server**: <BR>
* - /reaching_goal	
*
* **Description**: <BR>
* This node allows the user to:
* - Set a new goal.
* - Cancel the current goal.
* - Monitor when a goal is reached.
* - Publish the robot's position and velocity using a custom message.
* 
**/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>

#include <assignment_2_part1/state_pos_vel.h>		// including custom message
#include <nav_msgs/Odometry.h>				// including odometry message
#include <geometry_msgs/Point.h>			// including point message
#include <thread>					// include for the thread 

// GLOBAL VARIABLES ----------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Global variable for x-coordinate of the last goal.
 */
float x_last_goal;
/**
 * \brief Global variable for y-coordinate of the last goal.
 */
float y_last_goal;
/**
 * \brief Global variable that is 0 if the current goal has not been reached yet. it is used to print only once the 'goal reached' message.
 */
int printNum = 0;

/**
 * \brief Publisher for the robot state.
 */
ros::Publisher robot_state_pub;



// feedback_callback: CALLBACK FUNCTION ------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Callback function to handle feedback from the action server.
 * 
 * It prints a message when the robot is sufficiently close to the goal (within a threshold).
 *
 * \param feedback Pointer to the feedback message from the action server.
 */
void feedback_callback(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)
{
	// setting threshold
	float threshold = 0.5;
	if ((abs(feedback->actual_pose.position.x - x_last_goal) < threshold) && (abs(feedback->actual_pose.position.y - y_last_goal) < threshold) && (printNum == 0))
	{
		ROS_INFO("The target has been reached!");
		printNum++;
	}
}



// odometry_callback: CALLBACK FUNCTION ------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Callback function for the /odom topic.
 *
 * It publishes the robot's position and velocity using a custom message.
 *
 * \param msg Pointer to the odometry message containing the robot's position and velocity.
 */
void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	// creating the custom message
	assignment_2_part1::state_pos_vel state_message;
	
	// setting the message
  	state_message.x = msg->pose.pose.position.x;
    	state_message.y = msg->pose.pose.position.y;
    	state_message.vel_x = msg->twist.twist.linear.x;
    	state_message.vel_z = msg->twist.twist.angular.z;

    	// publishing the custom message
    	robot_state_pub.publish(state_message);
}


// MAIN ----------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Main function to run the action client node.
 *
 * This function initializes the ROS node, sets up publishers and subscribers,
 * creates an action client, and provides a user interface to set/cancel goals.
 * A thread is used to run the spin command. This allows the node to listen for incoming messages (and eventually handle callback functions), while the UI is running independently.
 *
 * \param argc Number of input arguments (if any).
 * \param argv Pointer to array of arguments (if any). 
 * \return 0 on successful execution
 */
int main (int argc, char **argv)
{
	ros::init(argc, argv, "action_server_client");
	ros::NodeHandle nh;
	
	// initializing the publisher for the robot state
	robot_state_pub = nh.advertise<assignment_2_part1::state_pos_vel>("/robot_state", 10);

  	// initializing the Subscriber for the /odom topic
  	ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odometry_callback);
  	
	// defining & initializing the publisher for the target coordinates
	ros::Publisher target_coord_pub = nh.advertise<geometry_msgs::Point>("/target_coordinates", 10);

	// creating the action server client
	actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("/reaching_goal", true);

	ROS_INFO("\n\nWaiting for action server to start.");
	// waiting for the action server to start
	ac.waitForServer(); //will wait for infinite time
  
  	// Creating a thread for ros::spin()
  	std::thread spinThread([]() 
  	{
      		ros::spin();
  	});
  	
	while (ros::ok())
	{
		ROS_INFO("\n\nEnter 1 to set a new goal,\nenter 2 to cancel the current goal,\nenter 0 to exit:\n");
  	
		// saving the user's answer in 'userChoice'
  		int userChoice; 
  		std::cin >> userChoice;
  		
  		// input control on user choice (1, 2, 3) 
  		if (std::cin.fail()) 
        	{
                	std::cin.clear();
                	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        		userChoice = 5;
        	}		
  		
  
  		if (userChoice == 1)	// new goal
  		{
  			// newGoal message creation
  			assignment_2_2024::PlanningGoal newGoal;
  	
  			printNum = 0;		
  			
  			// input control on chosen target coordinates
  						
  			while(true)
          		{		
             	 		// asking the user to set the new goal:coord x
  				ROS_INFO("\n\nSet the x coordinate of the new goal:\n");
  				std::cin >> newGoal.target_pose.pose.position.x;
  				
              			if (std::cin.fail()) 
                		{
                        		std::cin.clear();
                        		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        		ROS_INFO("Input not valid: enter a number.\n");

                		}
                		else
                		{
                    			break;
                		}
            		}
  			
  			while(true)
          		{		
             	 		// asking the user to set the new goal:coord y
  				ROS_INFO("\n\nSet the y coordinate of the new goal:\n");
  				std::cin >> newGoal.target_pose.pose.position.y;
  			
              			if (std::cin.fail()) 
                		{
                        		std::cin.clear();
                        		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        		ROS_INFO("Input not valid: enter a number.\n");

                		}
                		else
                		{
                    			break;
                		}
            		}
  		
  		
  			// updating global variables: goal coordinates
			x_last_goal = newGoal.target_pose.pose.position.x;
			y_last_goal = newGoal.target_pose.pose.position.y;	
			
			
			// defining the message for the target coordinates and 
			geometry_msgs::Point target_coord_msg;
			
			// initializing the message target_coord_msg
			target_coord_msg.x = newGoal.target_pose.pose.position.x;
			target_coord_msg.y = newGoal.target_pose.pose.position.y;
			
			// publishing the message target_coord_msg
    			target_coord_pub.publish(target_coord_msg);
    			

			// sending the message to the action server
  			ac.sendGoal(newGoal,
                  			actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>::SimpleDoneCallback(),
                            		actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>::SimpleActiveCallback(),
                            		feedback_callback);
  		}
  		else if (userChoice == 2)	// cancel the goal
  		{
  			ROS_INFO("\n\nThe current goal has been canceled. The robot will stop.\n");
  			ac.cancelGoal();
  		}
  		else if (userChoice == 0)	// exit
       	{
        		ROS_INFO("\n\nThe node is closing.");
        		break;
       	}
  		else
  		{
  			ROS_INFO("You entered a wrong input. Try again.\n");
  		}
	}

	//exit
  	return 0;
}
