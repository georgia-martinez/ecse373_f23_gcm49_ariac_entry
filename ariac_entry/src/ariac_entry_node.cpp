#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"

std::vector<osrf_gear::Order> orders;
osrf_gear::LogicalCameraImage::ConstPtr camera_images[10];
ros::ServiceClient locationClient; 
tf2_ros::Buffer tfBuffer;

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

bool startCompetition(ros::NodeHandle &n) {
	ROS_INFO("Waiting for the competition to start...");
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	begin_client.waitForExistence();

	std_srvs::Trigger begin_comp;

	int call_succeeded; // Capture service call success
	call_succeeded = begin_client.call(begin_comp); // Call the service

	if (!call_succeeded) {
		ROS_ERROR("Competition service call failed");
		return false;
	}
	
	if (!begin_comp.response.success) {
		ROS_ERROR("Competition service returned failure: %s", begin_comp.response.message.c_str());
		return false;
	}	

	ROS_INFO("Competition service called successfully");
	return true;
}

std::string getBinName(std::string type) {

	osrf_gear::GetMaterialLocations locationService;
	locationService.request.material_type = type;
	
	locationClient.call(locationService);
	
	for (osrf_gear::StorageUnit unit : locationService.response.storage_units){				
		return unit.unit_id;
	}

	return "NONE";
}

void moveArm(osrf_gear::Model model, std::string sourceFrame) {
	geometry_msgs::TransformStamped tfStamped;
	
	try {
		tfStamped = tfBuffer.lookupTransform("arm1_base_link", sourceFrame, ros::Time(0.0), ros::Duration(1.0));
		ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
		
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}
	
	// Create variables
	geometry_msgs::PoseStamped part_pose, goal_pose;

	// Copy pose from the logical camera.
	part_pose.pose = model.pose;
	tf2::doTransform(part_pose, goal_pose, tfStamped);
	
	goal_pose.pose.position.z += 0.10; // 10 cm above the part
	
	// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
	goal_pose.pose.orientation.w = 0.707;
	goal_pose.pose.orientation.x = 0.0;
	goal_pose.pose.orientation.y = 0.707;
	goal_pose.pose.orientation.z = 0.0;
	
	tf2::doTransform(part_pose, goal_pose, tfStamped);
	
	geometry_msgs::Point position = goal_pose.pose.position;
	ROS_WARN("Position: x=%f, y=%f, z=%f (relative to arm)", position.x, position.y, position.z);
}

void processOrder() {
	osrf_gear::Order order = orders.at(0);

	for(osrf_gear::Shipment shipment: order.shipments) {
		for(osrf_gear::Product product:shipment.products) {			
			std::string bin = getBinName(product.type);
			
			if (bin == "NONE") {
				continue;
			}
			
			for(int i = 0; i < 10; i++) {
				if(camera_topics[i].find(bin) == std::string::npos) {
					continue;
				}
				
				for (osrf_gear::Model model : camera_images[i]->models) {
					if (strstr(product.type.c_str(), model.type.c_str())) {
						geometry_msgs::Point position = model.pose.position;
						
						ROS_WARN("Model type: %s", product.type.c_str());
						ROS_WARN("Bin: %s", bin.c_str());
						ROS_WARN("Position: x=%f, y=%f, z=%f (relative to camera)", position.x, position.y, position.z); 
						
						std::string sourceFrame = "logical_camera_"+bin+"_frame";				
						moveArm(model, sourceFrame);
						
						break;
					}
				}	
			}
			
		}
	}

	orders.erase(orders.begin());
}

void orderCallback(const osrf_gear::Order msg) {
	orders.push_back(msg);
	ROS_INFO("Received: %s", msg.order_id.c_str());
	processOrder();
}

void cameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& image) {
	camera_images[index] = image;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ariac_entry");
	ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);
	
	// Define client for getting material locations
	locationClient = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	
	// Subscribe to orders
	ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderCallback);

	// Subscribe to camera topics	
	std::vector<ros::Subscriber> camera_subs;
	
	for(int i = 0; i < 10; i++) {
		camera_subs.push_back(n.subscribe<osrf_gear::LogicalCameraImage>(camera_topics[i], 1000, boost::bind(cameraCallback, i, _1)));
	}

	// Start the competition
	ros::Rate loop_rate(10);
	
	if(!startCompetition(n)) {
		return 1;
	}

	ros::spin();

	return 0;
}
