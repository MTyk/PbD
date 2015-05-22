/*
 * AssistanceController.cpp
 * 
 *  Handles the incremental learning. Provides assistance in two ways. 
 *  If PROACTIVE is false, stiffness will be very low during replaying the skill.
 *  If PROACTIVE is true, the orocos component ProactiveAssistance is started.
 *  No start with launchfile possible, as SigInts can't be handeled then.
 *  Created on: Apr 19, 2015
 *      Author: Martin Tykal
 */

#include "ros/ros.h"
#include "ros/console.h"

#include <boost/variant.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/process.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include "signal.h"

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "motion_control_msgs/JointPositions.h"

#include "trajectory/Joints.h"
#include "trajectory/Position.h"
#include "trajectory/Pose.h"
#include "trajectory/PoseForceZ.h"
#include "Imitation/Imitator.h"
#include "Imitation/BaseImitator.h"
#include "LWRLearning/LWRLearner.h"
#include "ExecutionInterface/Parameters.h"

#include "helpers/Exponential.h"
#include "helpers/PhaseOscillator.h"

#include <actionlib/client/simple_action_client.h>
#include "helpers/EmptyAction.h"
#include "helpers/friComm.h"

#include "KUKACommander/set_fri_ctrl.h"
#include "KUKACommander/set_bool.h"
#include "KUKACommander/joint_ptp_motion.h"
#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;
using namespace actionlib;
using namespace iros::pbd;
using namespace iros::pbd::data;
using namespace iros::pbd::dmp;
using namespace iros::pbd::helpers::canonical;
using namespace iros::pbd::dmp::learning;
using namespace iros::pbd::dmp::execution;

namespace cmd = ::boost::process;

#define PROACTIVE true

typedef enum {
	Start, Record, Learn, Execute, End, GravityComp, Replay
} States;
typedef Imitator<Joints, LWRLearner, sensor_msgs::JointState, std_msgs::Empty> JointsImitator;
typedef Imitator<Joints, LWRLearner, motion_control_msgs::JointPositions,
		std_msgs::Empty> JointsImitator2;
typedef Imitator<Position, LWRLearner, geometry_msgs::Point, std_msgs::Empty> PositionImitatorPoint;
typedef Imitator<Position, LWRLearner, geometry_msgs::Pose, std_msgs::Empty> PositionImitatorPose;
typedef Imitator<Pose, LWRLearner, geometry_msgs::Pose, std_msgs::Empty> PoseImitator;
typedef Imitator<PoseForceZ, LWRLearner, geometry_msgs::Pose,
		geometry_msgs::Vector3> PoseForceZImitator;

template<typename T>
T getParamDefault(NodeHandle& nh, string name, T defaultval);
int imitate(NodeHandle& nh, ros::ServiceClient& setControlModeClient,
		SimpleActionClient<EmptyAction>& execution_start_client,
		SimpleActionClient<EmptyAction>& execution_start_pos_client,
		SimpleActionClient<EmptyAction>& execution_stop);
unique_ptr<BaseImitator> initializeImitator(NodeHandle& nh);

void record_start_pos(const sensor_msgs::JointState::ConstPtr& jnt_state);
void assistanceStarted(const std_msgs::Empty msg);
void assistanceStopped(const std_msgs::Empty msg);

std_srvs::Empty empty;

int counter = 0;
int item_type = 0;
int item_msg_type = 0;
string bagfile = "stiffness";
string bagdir =
		"/home/intelligentrobotics/ws/pbd/Applications/bagfiles/experiments/assistance/";
string bagtopic;
string bagtopic2;

vector<double> start_pos;
bool start_pos_recorded = false;
bool cont = 0;
bool rec = 0;

ros::Publisher start_assistance;
ros::Publisher stop_assistance;
ros::Subscriber isstarted_assistance;
ros::Subscriber isstopped_assistance;

unique_ptr<BaseImitator> imitator;

int state = Start;

int main(int argc, char **argv) {
	ros::init(argc, argv, "AC");
	ROS_INFO("AssistanceController");

	//	signal(SIGTERM,sigHandle);
	ros::NodeHandle nh { "AssistanceController" };

	KUKACommander::set_bool set_bool_true;
	set_bool_true.request.activate = true;

	switch(item_type){
		//joints
		case 0:{
			bagtopic = "/iros/pbd/dmp/JointPos";
			bagtopic2 = "";
			break;
		}	
		//pose
		case 2:{
			bagtopic = "/iros/pbd/dmp/CartPose";
			bagtopic2 = "";
			break;
		}		
		//poseforce
		case 3:{
			bagtopic = "/iros/pbd/dmp/CartPose";
			bagtopic2 = "/iros/pbd/dmp/FTForce";
			break;
		}
	}	
	
	// Create service clients to communicate with the KUKACommander
	ros::ServiceClient activateGravityCompensation = nh.serviceClient<
			KUKACommander::set_bool>(
			"/KUKACommander/activateGravityCompensation");
	ros::ServiceClient stopGravityCompensation = nh.serviceClient<
			std_srvs::Empty>("/KUKACommander/stopGravityCompensation");
	ros::ServiceClient jointPTPMotion = nh.serviceClient<
			KUKACommander::joint_ptp_motion>("/KUKACommander/jointPTPMotion");
	// Setup client for KRC control mode service, wait for existence
	ros::ServiceClient setControlModeClient = nh.serviceClient<
			KUKACommander::set_fri_ctrl>("/KUKACommander/setControlMode");

	// ActionClient to communicate with Orocos (using the ROS executor interface)
	SimpleActionClient<EmptyAction> execution_start_client {
			"/iros/pbd/dmp/imitator/execution_start", true };
	SimpleActionClient<EmptyAction> execution_prepare_client {
			"/iros/pbd/dmp/imitator/execution_prepare", true };
	SimpleActionClient<EmptyAction> execution_start_pos_client {
			"/iros/pbd/dmp/imitator/execution_start_pos", true };
	SimpleActionClient<EmptyAction> execution_stop {
			"/iros/pbd/dmp/execution/interface/stop", true };

	// Topics to communicate with ProactiveAssistance		
	start_assistance = nh.advertise<std_msgs::Empty>(
			"/iros/pbd/assistanceStart", 1, true);
	stop_assistance = nh.advertise<std_msgs::Empty>("/iros/pbd/assistanceStop",
			1, true);
	isstarted_assistance = nh.subscribe<std_msgs::Empty>(
			"/iros/pbd/assistanceStarted", 1, assistanceStarted);
	isstopped_assistance = nh.subscribe<std_msgs::Empty>(
			"/iros/pbd/assistanceStopped", 1, assistanceStopped);
	bool run = true;

	while (run) {
		switch (state) {
		case Start: {
			ROS_INFO("======================================");
			ROS_INFO("    	Record new Skill? [1|0]");
			ROS_INFO("======================================");
			cin >> rec;
			if (rec) {
				state = Record;
			} else {
				state = Execute;
			}
			break;
		}
		case GravityComp: {
			activateGravityCompensation.call(set_bool_true);
			ROS_INFO("======================================");
			ROS_INFO("             Lift arm");
			ROS_INFO("         Finish with Enter");
			ROS_INFO("======================================");
			cin.ignore(1);
			stopGravityCompensation.call(empty);

			state = Execute;
			break;
		}
		case Record: {
			ros::AsyncSpinner spinner(2); // Use 2 threads
			spinner.start();
			KUKACommander::set_fri_ctrl control_pos_mon;
			control_pos_mon.request.control = FRI_CTRL_POSITION;
			control_pos_mon.request.state = FRI_STATE_MON;
			setControlModeClient.call(control_pos_mon);

			activateGravityCompensation.call(set_bool_true);
			ROS_INFO("======================================");
			ROS_INFO("     Move arm to start position");
			ROS_INFO("         Finish with Enter");
			ROS_INFO("======================================");
			cin.ignore(1);
			cin.ignore(1);
			{
				// Wait for first position and store it as start position
				Subscriber start_pos_recorder = nh.subscribe(
						"/iros/pbd/dmp/JointPos", 1, record_start_pos);
				while (!start_pos_recorded && ros::ok())
					;
			}
			if (PROACTIVE) {
				stopGravityCompensation.call(empty);
				start_assistance.publish(std_msgs::Empty { });
			}
			ROS_INFO("======================================");
			ROS_INFO("    		Record Trajectory");
			ROS_INFO("======================================");

			// Use rosbag to record the messages
			cmd::context ctx;
			ctx.stdin_behavior = cmd::inherit_stream();
			ctx.stdout_behavior = cmd::inherit_stream();
			ctx.environment = cmd::self::get_environment();

			// Create option string
			string rosbag_cmd = "rosbag record -o ";
			rosbag_cmd.append(bagdir);
			rosbag_cmd.append(bagfile);
			rosbag_cmd.append(" " + bagtopic);
			rosbag_cmd.append(" " + bagtopic2);

			signal(SIGINT, SIG_IGN);
			cmd::child rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
			ROS_INFO(" ");
			ROS_INFO("[Recording...]    <Stop with Ctrl+C>");
			ROS_INFO(" ");

			rosbag_shell.wait();
			if (!PROACTIVE)
				stopGravityCompensation.call(empty);
			if (PROACTIVE) {
				ROS_INFO("======================================");
				ROS_INFO("    	Record another demonstration?");
				ROS_INFO("======================================");
				bool moreDemo;
				cin >> moreDemo;
				if (moreDemo) {
					state = Record;
				} else {
					state = End;
				}
			} else {
				state = Execute;
			}
			spinner.stop();
			break;
		}
		case Execute: {
			ROS_INFO("Execute!");
			counter++;
			double stiffness;
			double damping;
			if (rec && !PROACTIVE) {
				//Increases stiffness with every run.
				stiffness = 20 + 1 * counter;
//				stiffness = 1000;
				damping = 0.7;
				if (stiffness > 2000) {
					stiffness = 2000;
				}
			} else {
				stiffness = 900;
				damping = 0.7;
			}
			ROS_INFO("Stiffness: %f, Damping: %f", stiffness, damping);
			//Writes stiffness and damping values to a textfile
			std::ofstream ofs(
					"/home/intelligentrobotics/ws/pbd/Applications/params/ComplianceParameters.txt",
					std::fstream::out);
			{
				boost::archive::text_oarchive oa(ofs);
				oa << stiffness;
				oa << damping;
			}
			ofs.close();

			//starts the imitation
			int success = imitate(nh, setControlModeClient,
					execution_start_client, execution_start_pos_client,
					execution_stop);
			if (success) {
				ROS_INFO("Imitate performed!");
				if (cont) {
					state = Execute;
					cont = 0;
				} else {
					state = End;
				}
			} else {
				ROS_WARN("Imitation could not be performed!");
				state = End;
			}
			break;
		}
		case End: {
			ROS_INFO("End");
			signal(SIGINT, SIG_DFL);
			run = false;
			stop_assistance.publish(std_msgs::Empty { });
			//This is necessary to avoid boost::LockError()
			imitator.reset();
			break;
		}
		}
	}

}
/**
 * Starts the imitation and records the movement of the robot during execution.
 * @param nh
 * @param setControlModeClient
 * @return
 */
int imitate(NodeHandle& nh, ros::ServiceClient& setControlModeClient,
		SimpleActionClient<EmptyAction>& execution_start_client,
		SimpleActionClient<EmptyAction>& execution_start_pos_client,
		SimpleActionClient<EmptyAction>& execution_stop) {
	ROS_INFO("Imitating");
	SimpleActionClient<EmptyAction> execution_prepare_client {
			"/iros/pbd/dmp/imitator/execution_prepare", true };

	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::ServiceClient jointPTPMotion = nh.serviceClient<
			KUKACommander::joint_ptp_motion>("/KUKACommander/jointPTPMotion");

	KUKACommander::set_fri_ctrl control_pos_mon;
	control_pos_mon.request.control = FRI_CTRL_POSITION;
	control_pos_mon.request.state = FRI_STATE_MON;
	if (counter == 1 && rec) {
		setControlModeClient.call(control_pos_mon);

		KUKACommander::joint_ptp_motion motion_srv;
		boost::array<double, 7> pos;
		for (size_t i = 0; i < 7; i++)
			pos[i] = start_pos[i];
		motion_srv.request.position = pos;
		motion_srv.request.speed = 100;
		// Move to start position
		jointPTPMotion.call(motion_srv);
	}

	imitator = initializeImitator(nh);
	imitator->learn(); // Learn trajectory parameters

	ROS_INFO("Finished learning!");

// Wait for ActionSevers to be setup

	if (!execution_start_client.waitForServer(ros::Duration(2.0))
			|| !execution_start_client.waitForServer(ros::Duration(2.0))
			|| !execution_start_client.waitForServer(ros::Duration(2.0))) {
		ROS_ERROR("Execution server not ready");
		return 0;
	}

// Call for preparation and wait
	ROS_INFO("Preparing...");
	EmptyGoal prepare_goal;
	execution_prepare_client.sendGoal(prepare_goal);
	if (!execution_prepare_client.waitForResult(ros::Duration(5.0))) {
		ROS_ERROR("Error while preparing execution");
		return 0;
	}

	ROS_INFO("Moving to start position...");
	EmptyGoal start_pos_goal;
	execution_start_pos_client.sendGoal(start_pos_goal);
	if (!execution_start_pos_client.waitForResult(ros::Duration(10.0))) {
		ROS_ERROR("Error while moving to start position");
		return 0;
	}

	ROS_INFO("Position reached.");

// Use rosbag to record the messages
	cmd::context ctx;
	ctx.stdin_behavior = cmd::inherit_stream();
	ctx.stdout_behavior = cmd::inherit_stream();
	ctx.environment = cmd::self::get_environment();
	cmd::child rosbag_shell = cmd::launch_shell("", ctx);
	if (rec) {

// Create option string
		string rosbag_cmd = "rosbag record -o ";
		rosbag_cmd.append(bagdir);
		rosbag_cmd.append(bagfile);
		rosbag_cmd.append(" " + bagtopic);
		rosbag_cmd.append(" " + bagtopic2);

		signal(SIGINT, SIG_IGN);
		rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
		ROS_INFO(" ");
		ROS_INFO("[Recording...]    <Stop with Ctrl+C>");
		ROS_INFO(" ");
	}
// Start execution and wait
	ROS_INFO("Switching mode and following learned trajectory");
	EmptyGoal start_goal;
	execution_start_client.sendGoal(start_goal);
	ROS_INFO("Executing...");
	if (!execution_start_client.waitForResult(ros::Duration(120.0))) {
		ROS_ERROR("Error while executing motion");
	} else {
		ROS_INFO("Execution finished!");
	}

	if (rec)
		ROS_INFO("Please stop recording");
	rosbag_shell.wait();

// Switch back to monitor mode

	if (!setControlModeClient.call(control_pos_mon)) {
		ROS_ERROR(
				"Cannot switch back to position control, KUKACommander not responding.");
	} else {
		ROS_INFO("Set to monitorstate");
	}

	if (rec) {
		ROS_INFO(" ");
		ROS_INFO("======================================");
		ROS_INFO("  Record another demonstration? [1|0]");
		ROS_INFO("======================================");
		cin >> cont;
	}
	if (!cont) {
		ROS_INFO(" ");
		ROS_INFO("======================================");
		ROS_INFO("  Replay learned skill? [1|0]");
		ROS_INFO("======================================");
		bool repl;
		cin >> repl;
		rec = !repl;
		if (repl)
			cont = 1;
	}

	signal(SIGINT, SIG_DFL);
	spinner.stop();
	return 1;

}
/**
 * Initializes the Imitator depening on item_type and item_msg_type. A global Imitator is important
 * to avoid errors in learning from second iteration on (demonstration already learned).
 * @param nh
 * @return
 */
unique_ptr<BaseImitator> initializeImitator(NodeHandle & nh) {
	
	double exp_start = 1.0;
	double exp_decay = 1.1;
	int num_kernels = 250;
	double stiffness = 2000;
	bool cyclic = false;
	
	
// Create canonical objects for discrete/cyclic motions
	Canonical* canonical;
	if (cyclic)
		canonical = new PhaseOscillator(exp_start);
	else
		canonical = new Exponential(exp_start, exp_decay);

// Ugly switch/case construct to create pointer to imitator object, depending on given parameters
	unique_ptr<BaseImitator> imitator { };
	switch (item_type) {
	case 0:
		switch (item_msg_type) {
		case 0: {
			imitator.reset(
					new JointsImitator(bagdir, bagfile, bagtopic, bagtopic2,
							num_kernels, canonical, stiffness, 250, cyclic));
			break;
		}
		case 1: {
			imitator.reset(
					new JointsImitator2(bagdir, bagfile, bagtopic, bagtopic2,
							num_kernels, canonical, stiffness, 250, cyclic));
			break;
		}
		default: {
			ROS_ERROR("Item type/msg_type combination not supported!");
			return 0;
		}
		}
		break;
	case 1:
		switch (item_msg_type) {
		case 2: {
			imitator.reset(
					new PositionImitatorPoint(bagdir, bagfile, bagtopic,
							bagtopic2, num_kernels, canonical, stiffness, 250,
							cyclic));
			break;
		}
		case 3: {
			imitator.reset(
					new PositionImitatorPose(bagdir, bagfile, bagtopic,
							bagtopic2, num_kernels, canonical, stiffness, 250,
							cyclic));
			break;
		}
		default: {
			ROS_ERROR("Item type/msg_type combination not supported!");
			return 0;
		}
		}
		break;
	case 2:
		if (item_msg_type != 3) {
			ROS_ERROR("Item type/msg_type combination not supported!");
			return 0;
		} else {
			imitator.reset(
					new PoseImitator(bagdir, bagfile, bagtopic, bagtopic2,
							num_kernels, canonical, stiffness, 250, cyclic));
			break;
		}
		break;
	case 3:
		if (item_msg_type != 3) {
			ROS_ERROR("Item type/msg_type combination not supported!");
			return 0;
		} else {
			imitator.reset(
					new PoseForceZImitator(bagdir, bagfile, bagtopic, bagtopic2,
							num_kernels, canonical, stiffness, 250, cyclic));
			break;
		}
		break;
	default:
		ROS_ERROR("Item type must be between 0 and 2");
		return 0;
	}
	return imitator;
}

void record_start_pos(const sensor_msgs::JointState::ConstPtr& jnt_state) {
	start_pos = jnt_state->position;
	start_pos_recorded = true;
}
//Callback functions
void assistanceStarted(const std_msgs::Empty msg) {
	ROS_INFO("Proactive Assistance started");
}
void assistanceStopped(const std_msgs::Empty msg) {
	ROS_INFO("Proactive Assistance stopped");
}
