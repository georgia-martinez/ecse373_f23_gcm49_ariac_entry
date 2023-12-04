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

#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

// name:= piston_rod_part x:=0.546343 y:=0.157368 z:=0.018169
// Position:              x=0.546975, y=0.157908, z=0.019435 (relative to camera) 

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

    //  Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.01);

    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    
    return joint_trajectory;	
}

trajectory_msgs::JointTrajectory getTrajectory(geometry_msgs::Point dest) {
	// Where is the end effector given the joint angles.
	// joint_states.position[0] is the linear_arm_actuator_joint
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];
	
	ur_kinematics::forward((double*) &q_pose, (double*) &T_pose);

	// What joint angles put the end effector at a specific place.
	// Desired pose of the end effector wrt the base_link.
	T_des[0][3] = (double) dest.x;
	T_des[1][3] = (double) dest.y;
	T_des[2][3] = (double) dest.z;
	T_des[3][3] = 1.0;
	
	// ROS_INFO("%f %f %f", T_des[0][3], T_des[1][3], T_des[2][3]);
	
	// The orientation of the end effector so that the end effector is down.
	T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
	T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
	
	int num_sols = ur_kinematics::inverse((double*) &T_des, (double*) &q_des);

	trajectory_msgs::JointTrajectory joint_trajectory = setupJointTrajectory();
	
	if (num_sols == 0) {
		ROS_WARN("NO INITIAL SOLUTIONS FOUND");
		joint_trajectory.header.frame_id = "empty";
        return joint_trajectory;
	} else {
		ROS_WARN("FOUND %d SOLUTIONS", num_sols);
	}
	
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
		
	// Find a good solution
	int solution_index = -1;
	
	for (int i = 0; i < num_sols; i++) {
		double shoulder_angle = q_des[i][1];
		double wrist_1_angle = q_des[i][3];

		if (abs(M_PI - wrist_1_angle) <= M_PI / 2 && shoulder_angle >= 4 * M_PI / 3) {
			solution_index = i;
			break;
		}
	}
	
	if (solution_index == -1) {
		ROS_WARN("NO VIABLE SOLUTIONS FOUND");
		joint_trajectory.header.frame_id = "empty";
        return joint_trajectory;
	}
	
	// Enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
        joint_trajectory.points[1].positions[indy + 1] = q_des[solution_index][indy];
    }

    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
	
	return joint_trajectory;
}

void callActionServer(trajectory_msgs::JointTrajectory joint_trajectory) {
	
	// Make sure there are solutions
    if (joint_trajectory.header.frame_id == "empty") {
        return;
    }
    
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
	trajectory_msgs::JointTrajectory joint_trajectory = getTrajectory(position);
	callActionServer(joint_trajectory);
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

void moveArmHome() {
	ROS_INFO("Returning home");

	geometry_msgs::Point home;
	home.x = -0.4;
	home.y = 0.1;
	home.z = 0.1;
	
	callActionServer(getTrajectory(home));
}

void moveArm(osrf_gear::Model model, std::string sourceFrame) {	
	geometry_msgs::TransformStamped tfStamped = getTfStamped(sourceFrame);
	
	geometry_msgs::PoseStamped part_pose; 
	geometry_msgs::PoseStamped goal_pose;

	// Copy pose from the logical camera
	part_pose.pose = model.pose;
	tf2::doTransform(part_pose, goal_pose, tfStamped);
	
	// goal_pose.pose.position.z += 0.10; // 10 cm above the part
	
	// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
	goal_pose.pose.orientation.w = 0.707;
	goal_pose.pose.orientation.x = 0.0;
	goal_pose.pose.orientation.y = 0.707;
	goal_pose.pose.orientation.z = 0.0;
	
	tf2::doTransform(part_pose, goal_pose, tfStamped);
	
	geometry_msgs::Point position = goal_pose.pose.position;
	ROS_WARN("Position: x=%f, y=%f, z=%f (relative to arm)", position.x, position.y, position.z);
	
	useGrabber(position, true);
}

void moveBase(double base_pos) {	
	trajectory_msgs::JointTrajectory joint_trajectory = setupJointTrajectory();
	
	//if (joint_trajectory.header.frame_id != "empty") {
		//ROS_INFO("Moving arm");
		//trajectory_pub.publish(joint_trajectory);
	//}
	
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
						
						moveBase(base_pos);
						// moveArm(model, sourceFrame);						
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
