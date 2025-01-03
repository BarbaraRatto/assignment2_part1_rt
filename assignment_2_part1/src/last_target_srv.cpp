#include "ros/ros.h"
#include "assignment_2_part1/target_coord.h"
#include "geometry_msgs/Point.h"


float updated_target_x = 0;
float updated_target_y = 0;


// curr_target_service_callback: CALLBACK FUNCTION --------------------------------------------------------------------------------------------------------

bool curr_target_service_callback(assignment_2_part1::target_coord::Request &req, assignment_2_part1::target_coord::Response &res)
{
    	res.x = updated_target_x;
    	res.y = updated_target_y;

    	return true;
}


// target_coords_callback: CALLBACK FUNCTION ---------------------------------------------------------------------------------------------------------------

void target_coords_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    	updated_target_x = msg->x;
    	updated_target_y = msg->y;
}


// MAIN ----------------------------------------------------------------------------------------------------------------------------------------------------

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
