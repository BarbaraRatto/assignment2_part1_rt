#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>

#include <assignment_2_part1/state_pos_vel.h>		// including custom message
#include <nav_msgs/Odometry.h>				// including odometry message
#include <geometry_msgs/Point.h>			// including point message

// GLOBAL VARIABLES ----------------------------------------------------------------------------------------------------------------------------------------------------

float x_last_goal;
float y_last_goal;
int printNum = 0;

// creating a publisher to publish the robot state
ros::Publisher robot_state_pub;



// feedback_callback: CALLBACK FUNCTION ------------------------------------------------------------------------------------------------------------------------

void feedback_callback(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)
{
	ros::spinOnce(); 		// Allow processing of subscriber callbacks
	// setting threshold
	float threshold = 0.5;
	if ((abs(feedback->actual_pose.position.x - x_last_goal) < threshold) && (abs(feedback->actual_pose.position.y - y_last_goal) < threshold) && (printNum == 0))
	{
		ROS_INFO("The target has been reached!");
		printNum++;
	}
}



// odometry_callback: CALLBACK FUNCTION ------------------------------------------------------------------------------------------------------------------------

// Callback for the /odom topic
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
  
	while (ros::ok())
	{
		ROS_INFO("\n\nEnter 1 to set a new goal,\nenter 2 to cancel the current goal,\nenter 0 to exit:\n");
  	
		// saving the user's answer in 'userChoice'
  		int userChoice; 
  		std::cin >> userChoice;
  
  		if (userChoice == 1)	// new goal
  		{
  			// newGoal message creation
  			assignment_2_2024::PlanningGoal newGoal;
  	
  			// asking the user to set the new goal:coord x
  			ROS_INFO("\n\nSet the x coordinate of the new goal:\n");
  			std::cin >> newGoal.target_pose.pose.position.x;		

			// asking the user to set the new goal:coord y
  			ROS_INFO("\n\nSet the y coordinate of the new goal:\n");
  			std::cin >> newGoal.target_pose.pose.position.y;
  		
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
