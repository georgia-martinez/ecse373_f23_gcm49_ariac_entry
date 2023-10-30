#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

tf2_ros::Buffer tfBuffer;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "...");
	ros::NodeHandle n;
    tf2_ros::TransformListener tfListener(tfBuffer);

	// Start the competition	
	ROS_INFO("Waiting for the competition to start...");
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	begin_client.waitForExistence();

	std_srvs::Trigger begin_comp;

	int service_call_succeeded; // Capture service call success
	service_call_succeeded = begin_client.call(begin_comp); // Call the service

	if (!service_call_succeeded) {
		ROS_ERROR("Competition service call failed");
		return 1;
	}
	
	if (!begin_comp.response.success) {
		ROS_ERROR("Competition service returned failure: %s", begin_comp.response.message.c_str());
		return 1;
	}	

	ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());

	ros::spin();

	return 0;
}
