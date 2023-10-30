#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"

std::vector<osrf_gear::Order> orders; // Tracks received orders 
ros::ServiceClient location_client; // 
tf2_ros::Buffer tf_buffer;

std::string camera_topics[] = {
	"/ariac/logical_camera_bin1",
	"/ariac/logical_camera_bin2",
	"/ariac/logical_camera_bin3",
	"/ariac/logical_camera_bin4",
	"/ariac/logical_camera_bin5",
	"/ariac/logical_camera_bin6",
	"/ariac/logical_camera_agv1",
	"/ariac/logical_camera_agv2",
	"/ariac/quality_control_sensor_1",
	"/ariac/quality_control_sensor_2"
};

void startCompetition(ros::NodeHandle &node) {
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
}

void orderCallback(const osrf_gear::Order msg) {
	orders.push_back(msg);
}

void cameraCallback(int index, const osrf_gear::LogicalCameraImage msg) {
	
} 

int main(int argc, char **argv) {

	ros::init(argc, argv, "ariac_entry");
	ros::NodeHandle n;

    tf2_ros::TransformListener tf_listener(tf_buffer);
	
	// Define client for getting material locations
	location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	
	// Subscribe to orders
	ros::Subscriber order_sub = n.subscribe("/ariac/orders", 1000, orderCallback);

	// Subscribe to camera topics	
	std::vector<ros::Subscriber> camera_subs;
	
	for(int i = 0; i < camera_topics.size(); i++) {
		camera_subs.push_back(n.subscribe<osrf_gear::LogicalCameraImage>(camera_topics[i], 1000, [=](const auto& msg) { cameraCallback(i, msg); }));
	}

	// Start the competition
	ros::Rate loop_rate(10);
	startCompetition(n);

	while(ros::ok()) {
		if (orders.isEmpty()) {
			continue;
		}
		
		osrf_gear::Order order = orders.at(0);
		
		
		orders.erase(orders.begin());
	}

	ros::spin();

	return 0;
}
