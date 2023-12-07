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

#include "ik_service/PoseIK.h" 
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>  // for M_PI

std::vector<osrf_gear::Order> orders;
osrf_gear::LogicalCameraImage::ConstPtr camera_images[10];
ros::ServiceClient locationClient; 
tf2_ros::Buffer tfBuffer;

sensor_msgs::JointState joint_states;

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_as;

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

// Instantiate variables for use with the kinematic system.
ros::ServiceClient ik_client;

double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

int count = 0;

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

trajectory_msgs::JointTrajectory setupJointTrajectory() {
	
	trajectory_msgs::JointTrajectory joint_trajectory;
    
    int count = 0;
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified

    // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.clear();

    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");
    
    // Set a start and end point.
    joint_trajectory.points.resize(2);
    
    // Set the start point to the current position of the joints from joint_states.
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}

	// When to start (immediately upon receipt).
	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
	
	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution
	joint_trajectory.points[1].positions[0] = joint_states.position[1];
    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
    
    return joint_trajectory;	
}

void callActionServer(trajectory_msgs::JointTrajectory joint_trajectory) {
	
	// Make sure there are solutions
    //if (joint_trajectory.header.frame_id == "empty") {
        //return;
    //}
    
	// Create the structure to populate for running the Action Server.
	control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
	
	// It is possible to reuse the JointTrajectory from above
	joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
	
	// The header and goal (not the tolerances) of the action must be filled out as well.
	// (rosmsg show control_msgs/FollowJointTrajectoryAction)
	joint_trajectory_as.action_goal.header.seq = count++;
	joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
	joint_trajectory_as.action_goal.header.frame_id = "/world";

	joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
	joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);

	actionlib::SimpleClientGoalState state = trajectory_as->sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
	ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
}

void useGrabber(geometry_msgs::Point position, bool grab) {
	//trajectory_msgs::JointTrajectory joint_trajectory = getTrajectory(position);
	//callActionServer(joint_trajectory);
}

geometry_msgs::TransformStamped getTfStamped(std::string frame) {
	geometry_msgs::TransformStamped tfStamped;
	
	try {	
		tfStamped = tfBuffer.lookupTransform("arm1_base_link", frame, ros::Time(0.0), ros::Duration(1.0));
		ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());	
		
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}
	
	return tfStamped;	
}

void moveArm(osrf_gear::Model model, std::string sourceFrame) {	

	// Copy pose from the logical camera
	geometry_msgs::TransformStamped tfStamped = getTfStamped(sourceFrame);
	
	geometry_msgs::PoseStamped part_pose; 
	geometry_msgs::PoseStamped goal_pose;

	part_pose.pose = model.pose;
	tf2::doTransform(part_pose, goal_pose, tfStamped);
	
	// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
	// goal_pose.pose.position.x = 0.0;
	// goal_pose.pose.position.y = 0.5;
	// goal_pose.pose.position.z = 0.1;
	
	goal_pose.pose.orientation.w = 0.707;
	goal_pose.pose.orientation.x = 0.0;
	goal_pose.pose.orientation.y = 0.707;
	goal_pose.pose.orientation.z = 0.0;	
	
	geometry_msgs::Point position = goal_pose.pose.position;
	geometry_msgs::Quaternion orientation = goal_pose.pose.orientation;
	
	ROS_WARN("Position: x=%f, y=%f, z=%f (relative to arm)", position.x, position.y, position.z);
	ROS_WARN("Orientation: w= %f, x=%f, y=%f, z=%f (relative to arm)", orientation.w, orientation.x, orientation.y, orientation.z);
	
	// Call the ik_service
	ik_service::PoseIK ik_pose; 
	ik_pose.request.part_pose = goal_pose.pose;	
    ik_client.call(ik_pose);
    
    int num_sols = ik_pose.response.num_sols;
    
    ROS_INFO("Number of poses returned [%i]", num_sols);
	
	if (num_sols == 0) {
		return;
	}
	
	trajectory_msgs::JointTrajectory joint_trajectory = setupJointTrajectory();	
	
	// Enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) {
      joint_trajectory.points[1].positions[indy+1] = 			 
      		ik_pose.response.joint_solutions[0].joint_angles[indy];
    }

    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

	callActionServer(joint_trajectory);
}

void moveBase(double base_pos) {	
	trajectory_msgs::JointTrajectory joint_trajectory = setupJointTrajectory();
	
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
        for (int indz = 0; indz < joint_states.name.size(); indz++) {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                joint_trajectory.points[1].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    
    joint_trajectory.points[1].positions[0] = base_pos;
    joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
    callActionServer(joint_trajectory);
}

void processOrder() {
	if (orders.size() == 0) {
		return;
	}

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
						
						double base_pos = 0; // base of arm
						
						if (bin == "bin4") {
							base_pos = .38;
						} else if (bin == "bin5") {
							base_pos = 1.15;
						} else if (bin == "bin6") {
							base_pos = 1.91;
						}
						
						ROS_INFO("Moving base to %s", bin.c_str());
						moveBase(base_pos);
						
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
}

void cameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& image) {
	camera_images[index] = image;
}

void jointCallback(const sensor_msgs::JointState msg) {
	joint_states = msg;
	
	std::string current;
	
	for (std::string s : joint_states.name) {
        current += s + " ";
    }
    
	ROS_INFO_STREAM_THROTTLE(10, current.c_str()); 
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ariac_entry");
	ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);
	
	// Define client for getting material locations
	locationClient = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	
	ros::Subscriber order_sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderCallback);

	// Subscribe to camera topics	
	std::vector<ros::Subscriber> camera_subs;
	
	for(int i = 0; i < 10; i++) {
		camera_subs.push_back(n.subscribe<osrf_gear::LogicalCameraImage>(camera_topics[i], 1000, boost::bind(cameraCallback, i, _1)));
	}
	
	ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);
    
    trajectory_as = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/ariac/arm1/arm/follow_joint_trajectory/", true);
	
	ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");

	// Start the competition

	if(!startCompetition(n)) {
		return 1;
	}

	ros::Rate loop_rate(10);
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(ros::ok) {
		processOrder();
		loop_rate.sleep();
	}

	return 0;
}
