/*
 * AssistanceController.cpp
 * 
 *  Two arguments: item_type and item_msg_type
 *  Joints: 0 0
 *  Cartesian: 2 3
 *  Cartesian with force: 3 3
 *  Handles the incremental learning. Three modes possible: Incremental without 
 *  proactive help, just proactive help and a combination of both.
 *  If PROACTIVE is false, stiffness will be very low during replaying the skill.
 *  If PROACTIVE is true, the orocos component ProactiveAssistance is started.
 *  If COMBINATION is true, the assistance controller will start the help
 *  and set its impedance.
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
#include "geometry_msgs/Twist.h"
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
#include "KUKACommander/cart_ptp_motion.h"
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
#define COMBINED true

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
int imitate(NodeHandle& nh,
		SimpleActionClient<EmptyAction>& execution_start_client,
		SimpleActionClient<EmptyAction>& execution_start_pos_client,
		SimpleActionClient<EmptyAction>& execution_stop);
unique_ptr<BaseImitator> initializeImitator(NodeHandle& nh);

void record_start_pos_jnt(const sensor_msgs::JointState::ConstPtr& jnt_state);
void record_start_pos_cart(const geometry_msgs::Pose::ConstPtr& cart_state);
void assistanceStarted(const std_msgs::Empty msg);
void assistanceStopped(const std_msgs::Empty msg);
void setImpedance(double stiffness, double damping);
void goToStart(NodeHandle& nh);

std_srvs::Empty empty;

int counter = 0;
int item_type;
int item_msg_type;
string bagfile = "stiffness";
string bagdir =
		"/home/intelligentrobotics/ws/pbd/Applications/bagfiles/experiments/assistance/";
string bagtopic;
string bagtopic2;
string bagtopic3;

vector<double> start_pos_jnt;
geometry_msgs::Pose start_pos_cart;
bool start_pos_recorded = false;
bool cont = 0;
bool rec = 0;

ros::Publisher start_assistance;
ros::Publisher stop_assistance;
ros::Publisher send_impedance;
ros::Subscriber isstarted_assistance;
ros::Subscriber isstopped_assistance;

unique_ptr<BaseImitator> imitator;

KUKACommander::set_fri_ctrl control_pos_mon;

int state = Start;

int main(int argc, char **argv) {
	ros::init(argc, argv, "AC");
	ROS_INFO("AssistanceController");
	if (argc == 3) {
		item_type = atoi(argv[1]);
		item_msg_type = atoi(argv[2]);
	} else {
		ROS_ERROR("Please give the item_type and item_msg_type as arguments!");
	}
	//	signal(SIGTERM,sigHandle);
	ros::NodeHandle nh { "AssistanceController" };

	KUKACommander::set_bool set_bool_true;
	set_bool_true.request.activate = true;

	control_pos_mon.request.control = FRI_CTRL_POSITION;
	control_pos_mon.request.state = FRI_STATE_MON;

	switch (item_type) {
	//joints
	case 0: {
		bagtopic = "/iros/pbd/dmp/JointPos";
		bagtopic2 = "";
		break;
	}
		//pose
	case 2: {
		bagtopic = "/iros/pbd/dmp/CartPose";
		bagtopic2 = "";
		break;
	}
		//poseforce
	case 3: {
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

	// Topics to communicate with ProactiveAssistance (created in ProactiveAssistance.ops)		
	start_assistance = nh.advertise<std_msgs::Empty>(
			"/iros/pbd/assistanceStart", 1, true);
	stop_assistance = nh.advertise<std_msgs::Empty>("/iros/pbd/assistanceStop",
			1, true);
	send_impedance = nh.advertise<geometry_msgs::Twist>("/iros/pbd/impedance",1,true);
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

			// if(!PROACTIVE)
			setControlModeClient.call(control_pos_mon);
			counter++;
			if (counter == 1) {
				activateGravityCompensation.call(set_bool_true);
				ROS_INFO("======================================");
				ROS_INFO("     Move arm to start position");
				ROS_INFO("         Finish with Enter");
				ROS_INFO("======================================");
				cin.ignore(1);
				cin.ignore(1);
				// Wait for first position and store it as start position
				Subscriber start_pos_recorder;
				// if (!PROACTIVE) {
					start_pos_recorder = nh.subscribe(
							"/iros/pbd/dmp/JointPos", 1, record_start_pos_jnt);
				// } else {
				// 	ROS_INFO("Saving start pos");
				// 	start_pos_recorder = nh.subscribe(
				// 			"/iros/pbd/dmp/CartPose", 1, record_start_pos_cart);
				// 	}

				while (!start_pos_recorded && ros::ok())
					;

				if (PROACTIVE) {
					stopGravityCompensation.call(empty);
					start_assistance.publish(std_msgs::Empty { });
				}
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
			rosbag_cmd.append(" " + bagtopic3);

			ROS_INFO("Rosbag_cmd: %s", rosbag_cmd.c_str());

			signal(SIGINT, SIG_IGN);
			cmd::child rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
			ROS_INFO(" ");
			ROS_INFO("[Recording...]    <Stop with Ctrl+C>");
			ROS_INFO(" ");

			rosbag_shell.wait();
			// if (!(PROACTIVE&&!COMBINED))
				stopGravityCompensation.call(empty);
			if (PROACTIVE && !COMBINED) {
				ROS_INFO("======================================");
				ROS_INFO(" Record another demonstration? [1|0]");
				ROS_INFO("======================================");
				bool moreDemo;
				cin >> moreDemo;
				if (moreDemo) {
					state = Record;
					goToStart(nh);
				} else {
					ROS_INFO(" ");
					ROS_INFO("======================================");
					ROS_INFO("  Replay learned skill? [1|0]");
					ROS_INFO("======================================");
					bool repl;
					cin >> repl;
					if (repl) {
						state = Execute;
					} else {
						state = End;
					}
				}
			} else {
				state = Execute;
			}
			spinner.stop();
			break;
		}
		case Execute: {
			ROS_INFO("Execute!");
			//dont increase counter if only proactive is running
			if (PROACTIVE == COMBINED)
				counter++;
			//starts the imitation
			int success = imitate(nh, execution_start_client,
					execution_start_pos_client, execution_stop);
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
			ros::ServiceClient setControlModeClient = nh.serviceClient<
					KUKACommander::set_fri_ctrl>(
					"/KUKACommander/setControlMode");
			control_pos_mon.request.control = FRI_CTRL_POSITION;
			control_pos_mon.request.state = FRI_STATE_MON;
			setControlModeClient.call(control_pos_mon);
			run = false;
			setImpedance(1000, 0.7);
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
int imitate(NodeHandle& nh,
		SimpleActionClient<EmptyAction>& execution_start_client,
		SimpleActionClient<EmptyAction>& execution_start_pos_client,
		SimpleActionClient<EmptyAction>& execution_stop) {
	ROS_INFO("Imitating");
	SimpleActionClient<EmptyAction> execution_prepare_client {
			"/iros/pbd/dmp/imitator/execution_prepare", true };

	setImpedance(1000, 0.7);

	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();

	ROS_INFO("Counter: %d", counter);
	if (PROACTIVE) {
		stop_assistance.publish(std_msgs::Empty { });

	}
	/*if (counter == 2 && rec) {
		goToStart(nh);
	}*/

	imitator = initializeImitator(nh);
	imitator->learn(); // Learn trajectory parameters

	ROS_INFO("Finished learning!");

// Wait for ActionSevers to be setup

	if (!execution_start_client.waitForServer(ros::Duration(2.0))
			|| !execution_prepare_client.waitForServer(ros::Duration(2.0))
			|| !execution_start_pos_client.waitForServer(ros::Duration(2.0))) {
		ROS_ERROR("Execution server not ready");
		spinner.stop();
		return 0;
	}
	ros::ServiceClient setControlModeClient = nh.serviceClient<
			KUKACommander::set_fri_ctrl>("/KUKACommander/setControlMode");
	control_pos_mon.request.control = FRI_CTRL_POSITION;
	control_pos_mon.request.state = FRI_STATE_MON;
	setControlModeClient.call(control_pos_mon);

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
	if (!execution_start_pos_client.waitForResult(ros::Duration(20.0))) {
		ROS_ERROR("Error while moving to start position");
		return 0;
	}
	//Due to the motion to the start position, FRI goes into JNT_IMP, but we need CART_IMP for COMBINED
	if(PROACTIVE&&COMBINED){
		ros::ServiceClient setControlModeClient = nh.serviceClient<
		KUKACommander::set_fri_ctrl>("/KUKACommander/setControlMode");
		KUKACommander::set_fri_ctrl control_cart_imp;

		control_cart_imp.request.control = FRI_CTRL_CART_IMP;
		control_cart_imp.request.state = FRI_STATE_CMD;
		setControlModeClient.call(control_cart_imp);
	}

	if (rec)
		setImpedance(30 + 5 * counter, 0.7);

	ROS_INFO("Position reached.");

	if(PROACTIVE&&rec)
		start_assistance.publish(std_msgs::Empty { });
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

		ROS_INFO("Rosbag_cmd: %s", rosbag_cmd.c_str());
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
	double exp_decay = 3.0;
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
	ROS_INFO("Item_type: %d Item_msg_type: %d", item_type, item_msg_type);
	switch (item_type) {
	case 0:
		switch (item_msg_type) {
		case 0: {
			imitator.reset(
					new JointsImitator(bagdir, bagfile, bagtopic, bagtopic2,
							num_kernels, canonical, stiffness, 250, cyclic));
			ROS_INFO("JointsImitator created");
			break;
		}
		case 1: {
			imitator.reset(
					new JointsImitator2(bagdir, bagfile, bagtopic, bagtopic2,
							num_kernels, canonical, stiffness, 250, cyclic));
			ROS_INFO("JointsImitator2 created");
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
			ROS_INFO("PositionImitatorPoint created");
			break;
		}
		case 3: {
			imitator.reset(
					new PositionImitatorPose(bagdir, bagfile, bagtopic,
							bagtopic2, num_kernels, canonical, stiffness, 250,
							cyclic));
			ROS_INFO("PositionImitatorPose created");
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
			ROS_INFO("PoseImitator created");
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
			ROS_INFO("PoseForceZImitator created");
			break;
		}
		break;
	default:
		ROS_ERROR("Item type must be between 0 and 2");
		return 0;
	}
	return imitator;
}

void record_start_pos_jnt(const sensor_msgs::JointState::ConstPtr& jnt_state) {
	start_pos_jnt = jnt_state->position;
	start_pos_recorded = true;
}
void record_start_pos_cart(const geometry_msgs::Pose::ConstPtr& cart_state) {
	ROS_INFO("Start_pos_rec");
	start_pos_cart.position = cart_state->position;
	start_pos_cart.orientation = cart_state->orientation;
	start_pos_recorded = true;
}
//Callback functions
void assistanceStarted(const std_msgs::Empty msg) {
	ROS_INFO("Proactive Assistance started");
}
void assistanceStopped(const std_msgs::Empty msg) {
	ROS_INFO("Proactive Assistance stopped");
}
void setImpedance(double stiffness, double damping) {
	if (!rec && PROACTIVE &&!COMBINED) {
		stiffness = 0;
		damping = 0.7;
	}
	geometry_msgs::Twist impedance;

	impedance.linear.x = stiffness;
	impedance.linear.y = stiffness;
	impedance.linear.z = stiffness;
	impedance.angular.x = damping;
	impedance.angular.y = damping;	
	impedance.angular.z = damping;	
	
	send_impedance.publish(impedance);

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

}
void goToStart(NodeHandle& nh) {
	ROS_INFO("Go to start");
	ros::ServiceClient setControlModeClient = nh.serviceClient<
			KUKACommander::set_fri_ctrl>("/KUKACommander/setControlMode");

	// if (!PROACTIVE) {
		setControlModeClient.call(control_pos_mon);
		ros::ServiceClient jointPTPMotion = nh.serviceClient<
				KUKACommander::joint_ptp_motion>(
				"/KUKACommander/jointPTPMotion");

		KUKACommander::joint_ptp_motion motion_srv;
		boost::array<double, 7> pos;
		for (size_t i = 0; i < 7; i++)
			pos[i] = start_pos_jnt[i];
		motion_srv.request.position = pos;
		motion_srv.request.speed = 100;
		// Move to start position
		jointPTPMotion.call(motion_srv);
/*	} else {
		ros::ServiceClient cartPTPMotion = nh.serviceClient<
				KUKACommander::cart_ptp_motion>(
				"/KUKACommander/CartesianPTPMotion");
		KUKACommander::cart_ptp_motion motion_srv;
		motion_srv.request.position = start_pos_cart;
		motion_srv.request.speed = 100;
		cartPTPMotion.call(motion_srv);
	}*/
}
