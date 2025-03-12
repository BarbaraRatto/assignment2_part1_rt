/**
* \file last_target_srv.cpp
* \brief This node implements a service that returns the coordinates of the last target set by the user.
* \author Barbara Ratto
* \version 1.0
* \date 12/03/25
*
* \details
*
* **Subscribes to**: <BR>
* - /target_coordinates
*	
* **Service**: <BR>
* - last_target_coord	
*
* **Description**: <BR>
* This service node reads from the topic '/target_coordinates' the coordinates of the last target set by the user.
* This topic is updated by the node 'action_server_client.cpp' each time that a new target is set.
* When the service is called, it returns the latest values of the goal reading them from the always up to date global variables.
**/

#include "ros/ros.h"
#include "assignment_2_part1/target_coord.h"
#include "geometry_msgs/Point.h"

// GLOBAL VARIABLES ----------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Global variable for x-coordinate of the last updated target.
 */
float updated_target_x = 0;
/**
 * \brief Global variable for y-coordinate of the last updated target.
 */
float updated_target_y = 0;


// curr_target_service_callback: CALLBACK FUNCTION --------------------------------------------------------------------------------------------------------

/**
 * \brief Callback function for the last_target_coord service.
 *
 * This function assigns the last known target coordinates to the response message.
 *
 * \param req The service request (not used in this implementation).
 * \param res The service response, which contains the last target coordinates.
 * \return True upon successful execution.
 *
 */

bool curr_target_service_callback(assignment_2_part1::target_coord::Request &req, assignment_2_part1::target_coord::Response &res)
{
    	res.x = updated_target_x;
    	res.y = updated_target_y;

    	return true;
}

// target_coords_callback: CALLBACK FUNCTION ---------------------------------------------------------------------------------------------------------------

/**
 * \brief Callback function for the /target_coordinates topic.
 *
 * This function updates the global variables `updated_target_x` and `updated_target_y`
 * whenever new target coordinates are published on `/target_coordinates`.
 
 * \param msg Pointer to the received message containing the new target coordinates.
 *
 */


void target_coords_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    	updated_target_x = msg->x;
    	updated_target_y = msg->y;
}


// MAIN ----------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Main function to initialize and run the last target service node.
 *
 * This function initializes the ROS node, creates a service for retrieving the last target coordinates,
 * subscribes to the `/target_coordinates` topic, and keeps the node running with `ros::spin()`.
 * 
 * \param argc Number of input arguments (if any).
 * \param argv Pointer to array of arguments (if any).
 * \return 0 on successful execution.
 *
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "last_target_service_node");
    	ros::NodeHandle nh;

	// service that returns the target coordinates
    	ros::ServiceServer target_coordinates_service = nh.advertiseService("last_target_coord", curr_target_service_callback);

    	// Subscriber to the /target_coordinates topic
    	ros::Subscriber target_coordinates_sub = nh.subscribe("/target_coordinates", 10, target_coords_callback);

    	ros::spin();
    	return 0;
}
