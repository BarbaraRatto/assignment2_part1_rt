#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>



int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_server_client");
  ros::NodeHandle nh;

  // creating the action server client
  actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("/reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // waiting for the action server to start
  ac.waitForServer(); //will wait for infinite time
  
  while (ros::ok())
  {
  	ROS_INFO("\nEnter 1 to set a new goal,\nenter 2 to cancel the current goal,\nenter 0 to exit:\n");
  	
  	// saving the user's answer in 'userChoice'
  	int userChoice; 
  	std::cin >> userChoice;
  
  	if (userChoice == 1)	// new goal
  	{
  		// newGoal message creation
  		assignment_2_2024::PlanningGoal newGoal;
  	
  		// asking the user to set the new goal:coord x
  		ROS_INFO("\nSet the x coordinate of the new goal:\n");
  		std::cin >> newGoal.target_pose.pose.position.x;

		// asking the user to set the new goal:coord y
  		ROS_INFO("\nSet the y coordinate of the new goal:\n");
  		std::cin >> newGoal.target_pose.pose.position.y;

		// sending the message to the action server
  		ac.sendGoal(newGoal);
  	}
  	else if (userChoice == 2)	// cancel the goal
  	{
  		ROS_INFO("\nThe current goal has been canceled. The robot will stop.\n");
  		ac.cancelGoal();
  	}
  	else if (userChoice == 0)	// exit
        {
        	ROS_INFO("\nThe node is closing.");
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
