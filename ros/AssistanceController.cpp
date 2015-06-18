
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
#include "rosbag/bag.h"
#include "rosbag/view.h"	 

#include <boost/variant.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/process.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector> 
#include "signal.h"
#include <dirent.h>


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

struct Index {
        int x;   // x coordinate
        int y;   // y coordinate
};
struct dirent *ent; //To go through a directory

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
vector<vector<double> > calculateMatrix(vector<double> s, vector<double> t);
double calculateDistance(vector<vector<double> > matrix);
vector<Index> calculateWarppath(vector<vector<double> > matrix);
void dynamicTimeWarp(int counter);
void trimVector(std::vector<double> &vec, size_t start, size_t end);
double normVector(double x, double y, double z);

std_srvs::Empty empty;

int counter = 0;
int item_type;
int item_msg_type;
//typename msg_typename1=geometry_msgs::Pose;
//typename msg_typename2;
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
        //      signal(SIGTERM,sigHandle);
        ros::NodeHandle nh { "AssistanceController" };

        KUKACommander::set_bool set_bool_false;
        set_bool_false.request.activate = false;

        control_pos_mon.request.control = FRI_CTRL_POSITION;
        control_pos_mon.request.state = FRI_STATE_MON;

        switch (item_type) {
        //joints
        case 0: {
                bagtopic = "/iros/pbd/dmp/JointPos";
                bagtopic2 = "";
               // msg_typename1 = sensor_msgs::JointState;
                break;
        }
                //pose
        case 2: {
                bagtopic = "/iros/pbd/dmp/CartPose";
                bagtopic2 = "";
               // msg_typename1 = geometry_msgs::Pose;
                break;
        }
                //poseforce
        case 3: {
                bagtopic = "/iros/pbd/dmp/CartPose";
                bagtopic2 = "/iros/pbd/dmp/FTForce";
                // msg_typename1 = geometry_msgs::Pose;
                // msg_typename2 = geometry_msgs::Vector3;
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
                        ROS_INFO("      Record new Skill? [1|0]");
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
                        activateGravityCompensation.call(set_bool_false);
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
                                activateGravityCompensation.call(set_bool_false);
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
                                //      ROS_INFO("Saving start pos");
                                //      start_pos_recorder = nh.subscribe(
                                //                      "/iros/pbd/dmp/CartPose", 1, record_start_pos_cart);
                                //      }

                                while (!start_pos_recorded && ros::ok())
                                        ;

                                if (PROACTIVE) {
                                        stopGravityCompensation.call(empty);
                                        start_assistance.publish(std_msgs::Empty { });
                                }
                        }
                        ROS_INFO("======================================");
                        ROS_INFO("              Record Trajectory");
                        ROS_INFO("======================================");

                        // Use rosbag to record the messages
                        cmd::context ctx;
                        ctx.stdin_behavior = cmd::inherit_stream();
                        ctx.stdout_behavior = cmd::inherit_stream();
                        ctx.environment = cmd::self::get_environment();

                        // Create option string
                        stringstream cmd_stream;
                        string rosbag_cmd = "rosbag record -o ";
                        cmd_stream << rosbag_cmd << bagdir << bagfile << "_" << counter << " " << bagtopic << " "<< bagtopic2;
                        /*rosbag_cmd.append(bagdir);
                        rosbag_cmd.append(bagfile+"_"+counter;
                        rosbag_cmd.append(" " + bagtopic);
                        rosbag_cmd.append(" " + bagtopic2);*/
                        rosbag_cmd = cmd_stream.str();
                        // string rosbag_cmd = "rosbag record -o ";
                        // rosbag_cmd.append(bagdir);
                        // rosbag_cmd.append(bagfile);
                        // rosbag_cmd.append(" " + bagtopic);
                        // rosbag_cmd.append(" " + bagtopic2);
                        // rosbag_cmd.append(" " + bagtopic3);

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

        ros::AsyncSpinner spinner(3); // Use 3 threads
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
                setImpedance(10 + 5 * counter, 0.7);

        ROS_INFO("Position reached.");

        if(PROACTIVE&&rec)
                start_assistance.publish(std_msgs::Empty { });
// Use rosbag to record the messages
        cmd::context ctx;
        ctx.stdin_behavior = cmd::inherit_stream();
        ctx.stdout_behavior = cmd::inherit_stream();
        ctx.environment = cmd::self::get_environment();
        cmd::child rosbag_shell = cmd::launch_shell("", ctx);
        cmd::child combine_shell = cmd::launch_shell("", ctx);
        if (rec) {
        		stringstream cmd_stream;
                string rosbag_cmd = "rosbag record -o ";
                cmd_stream << rosbag_cmd << bagdir << bagfile << "_" << counter << " " << bagtopic << " "<< bagtopic2;
                rosbag_cmd = cmd_stream.str();
                ROS_INFO("Rosbag_cmd: %s", rosbag_cmd.c_str());
                signal(SIGINT, SIG_IGN);
                rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);

                // stringstream cmd_stream2;
                // string rosbag_cmd2 = "rosbag record -o ";
                // cmd_stream2 << rosbag_cmd2 << bagdir << bagfile << "_combined" << " " << "/iros/pbd/desCartPos";
                // rosbag_cmd2 = cmd_stream2.str();
                // ROS_INFO("Rosbag_cmd: %s", rosbag_cmd2.c_str());
                // combine_shell = cmd::launch_shell(rosbag_cmd2,ctx);
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
        /*combine_shell.wait();


        //getting the files from the bag folder
        vector<string> filenames;
	    DIR *dir;
	    if ((dir = opendir (bagdir.c_str())) != NULL) {
	        while ((ent = readdir (dir)) != NULL) {
	            filenames.push_back(ent->d_name);
	        }
	        closedir (dir);
	    }
	    //searching for the previous two bags and the combined one
	    stringstream last_bag_stream;
	    last_bag_stream << "stiffness_"<< counter-1;
	    stringstream obsolete_bag_stream;
	    obsolete_bag_stream << "stiffness_" << counter-2;
	    stringstream combined_bag_stream;
	    combined_bag_stream << "stiffness_combined_201";
	    string last_bag_full_name;
	    string obsolete_bag_full_name;
	    string combined_bag_full_name;

        rosbag::Bag combined_bag;
	    for (string s : filenames){
	    	if(s.find(last_bag_stream.str())!=string::npos){
	    		last_bag_full_name = bagdir + s;
	    	}
	    	if(s.find(obsolete_bag_stream.str())!=string::npos){
	    		obsolete_bag_full_name = bagdir + s;
	    	}
	    	if(s.find(combined_bag_stream.str())!=string::npos){
	    		combined_bag_full_name = bagdir +s;
	            combined_bag.open(combined_bag_full_name);
	    	}

	    }
	    ROS_INFO("Bagfiles found");

	    // The combined bagfile has the wrong topic. Rosbag does not allow to change a bagfile
	    // You can only read or write. Thus a new bag (transformed_combined_bag) is created where the
	    // pose is stored with the right topic.
        string transformed_combined_bag_dir = bagdir + bagfile + "_combined.bag";
        rosbag::Bag transformed_combined_bag(transformed_combined_bag_dir, rosbag::bagmode::Write);
        ROS_INFO("Opening Bagfiles...");

        rosbag::TopicQuery query = rosbag::TopicQuery("/iros/pbd/desCartPos");
        rosbag::View view(combined_bag, query);

        ROS_INFO("Transforming combined bagfile");
        boost::shared_ptr<geometry_msgs::Pose> message;
        geometry_msgs::Pose combined_pose;
        for( rosbag::MessageInstance m : view){
        	message = m.instantiate<geometry_msgs::Pose>();
        	combined_pose.position = message->position;
        	combined_pose.orientation = message->orientation;
        	transformed_combined_bag.write(bagtopic, m.getTime(), combined_pose);
        }
	
		combined_bag.close();
		transformed_combined_bag.close();
		ROS_INFO("Transformed");
		//Overwriting the last bagfile with the combined one and removing unnecessary files
        cmd::child mv_shell = cmd::launch_shell("", ctx);
	    string mv_cmd = "mv -f " +  transformed_combined_bag_dir + " " + last_bag_full_name 
	    			+ "; rm " + combined_bag_full_name +"; rm " + obsolete_bag_full_name;
	    ROS_INFO("MV-Cmd: %s", mv_cmd.c_str());
	    mv_shell = cmd::launch_shell(mv_cmd, ctx);
*/
        //DTW
        // if(rec)
        //      dynamicTimeWarp(counter);

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
        if(stiffness > 2000){
        	stiffness = 2000;
        }
        if(damping > 1.0){
        	damping = 1.0;
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
/*      } else {
                ros::ServiceClient cartPTPMotion = nh.serviceClient<
                                KUKACommander::cart_ptp_motion>(
                                "/KUKACommander/CartesianPTPMotion");
                KUKACommander::cart_ptp_motion motion_srv;
                motion_srv.request.position = start_pos_cart;
                motion_srv.request.speed = 100;
                cartPTPMotion.call(motion_srv);
        }*/
}
vector<vector<double> > calculateMatrix(vector<double> s, vector<double> t) {
	vector<vector<double> > dtw(s.size(), vector<double>(t.size()));
	for (size_t i = 0; i < s.size(); i++) {
		for (size_t j = 0; j < t.size(); j++) {
			dtw[i][j] = HUGE_VAL;
		}
	}
	dtw[0][0] = 0;
	for (size_t i = 0; i < s.size(); i++) {
		for (size_t j = 0; j < t.size(); j++) {
			double minimum = HUGE_VAL;
			double dist = abs(s[i] - t[j]);
			if (i > 0)
				minimum = dtw[i - 1][j];
			if (j > 0)
				minimum = min(minimum, dtw[i][j - 1]);
			if ((i > 0) && (j > 0))
				minimum = min(minimum, dtw[i - 1][j - 1]);
			if ((i == 0) && (j == 0))
				dtw[i][j] = fabs(s[i] - t[j]);
			else
				dtw[i][j] = minimum + dist;
		}
	}
	return dtw;
}
double calculateDistance(vector<vector<double> > matrix) {
	int length = matrix.size();
	int hight = matrix[0].size();
	double dist = matrix[length - 1][hight - 1];
	return dist;

}

vector<Index> calculateWarppath(vector<vector<double> > matrix) {

	vector<Index> warpPath;
	int i = matrix.size() - 1;
	int j = matrix[0].size() - 1;

	while (true) {
		if (i == 0 && j == 0)
			break;
		if (i == 0) {
			j--;
		} else {
			if (j == 0)
				i--;
			else {
				double v = HUGE_VAL;
				int index = 0;
				if (matrix[i - 1][j] < v) {
					v = matrix[i - 1][j];
					index = 1;
				}
				if (matrix[i][j - 1] < v) {
					v = matrix[i][j - 1];
					index = 2;
				}
				if (matrix[i - 1][j - 1] <= v) {
					index = 3;
				}
				switch (index) {
				case (1):
					i--;
					break;
				case (2):
					j--;
					break;
				case (3):
					i--;
					j--;
					break;
				}
			}
		}
		Index pos;
		pos.x = i;
		pos.y = j;
		warpPath.push_back(pos);
	}
	return warpPath;
}

void dynamicTimeWarp(int counter){
    stringstream bag_tag1,bag_tag2;
    bag_tag1 << "stiffness_"<< counter-1;
    bag_tag2 << "stiffness_"<< counter;

    cout << "bag_name1: " << bag_tag1.str() << "bag_name2: " << bag_tag2.str() << endl;

    vector<string> filenames;
    DIR *dir;
    if ((dir = opendir (bagdir.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            filenames.push_back(ent->d_name);
            printf ("%s\n", ent->d_name);
        }
        closedir (dir);
    }

    rosbag::Bag bag1;
    rosbag::Bag bag2;
    for (string s : filenames){
        if(s.find(bag_tag1.str())!=string::npos){
            stringstream bagfile1;
            bagfile1 << bagdir << s;
            bag1.open(bagfile1.str());
        }else if (s.find(bag_tag2.str())!=string::npos){
            stringstream bagfile2;
            bagfile2 << bagdir << s;
            bag2.open(bagfile2.str());
        }
    }
    string bag_name1= bag1.getFileName();
    string bag_name2= bag2.getFileName();

    rosbag::TopicQuery query("");
    query = rosbag::TopicQuery(bagtopic);

    rosbag::View view1(bag1, query);
    rosbag::View view2(bag2, query);

    std::vector<double> t1_x, t1_y, t1_z, t1_a, t1_b, t1_c, t1_d;
    std::vector<double> t2_x, t2_y, t2_z, t2_a, t2_b, t2_c, t2_d;
    std::vector<ros::Time> time1;
    std::vector<ros::Time> time2;
    std::vector<double> t1_x_dtw, t1_y_dtw, t1_z_dtw, t1_a_dtw, t1_b_dtw, t1_c_dtw, t1_d_dtw;
    std::vector<double> t2_x_dtw, t2_y_dtw, t2_z_dtw, t2_a_dtw, t2_b_dtw, t2_c_dtw, t2_d_dtw;
    std::vector<ros::Time> time1_dtw;
    std::vector<ros::Time> time2_dtw;

    boost::shared_ptr<geometry_msgs::Pose> message1;
    for(rosbag::MessageInstance const m : view1){
            message1 = m.instantiate<geometry_msgs::Pose>();
            t1_x.push_back(message1->position.x);
            t1_y.push_back(message1->position.y);
            t1_z.push_back(message1->position.z);
            t1_a.push_back(message1->orientation.x);
            t1_b.push_back(message1->orientation.y);
            t1_c.push_back(message1->orientation.z);
            t1_d.push_back(message1->orientation.w);
            time1.push_back(m.getTime());
    }

    boost::shared_ptr<geometry_msgs::Pose> message2;
    for(rosbag::MessageInstance const m : view2){
            message2 = m.instantiate<geometry_msgs::Pose>();
            t2_x.push_back(message2->position.x);
            t2_y.push_back(message2->position.y);
            t2_z.push_back(message2->position.z);
            t2_a.push_back(message1->orientation.x);
            t2_b.push_back(message1->orientation.y);
            t2_c.push_back(message1->orientation.z);
            t2_d.push_back(message1->orientation.w);
            time2.push_back(m.getTime());
    }

    ROS_INFO("Bagsize 1: %lu",bag1.getSize());
    ROS_INFO("Bagsize 2: %lu",bag2.getSize());
    ROS_INFO("t1_x size: %lu",t1_x.size());
    ROS_INFO("t2_x size: %lu",t2_x.size());

    vector< vector<double> > distMatrix = calculateMatrix(t1_x,t2_x);
    ROS_INFO("Distance: %f",calculateDistance(distMatrix) );
    vector<Index> warpPath = calculateWarppath(distMatrix);
    cout << warpPath[0].x << endl; 

    std::reverse(warpPath.begin(),warpPath.end());
    ROS_INFO("reversed");
    t1_x_dtw.resize(warpPath.size());
    t1_y_dtw.resize(warpPath.size());
    t1_z_dtw.resize(warpPath.size());
    t1_a_dtw.resize(warpPath.size());
    t1_b_dtw.resize(warpPath.size());
    t1_c_dtw.resize(warpPath.size());
    t1_d_dtw.resize(warpPath.size());
    time1_dtw.resize(warpPath.size());

    t2_x_dtw.resize(warpPath.size());
    t2_y_dtw.resize(warpPath.size());
    t2_z_dtw.resize(warpPath.size());
    t2_a_dtw.resize(warpPath.size());
    t2_b_dtw.resize(warpPath.size());
    t2_c_dtw.resize(warpPath.size());
    t2_d_dtw.resize(warpPath.size());
    time2_dtw.resize(warpPath.size());

    for(size_t i = 0;i < warpPath.size(); i++){
            t1_x_dtw[i] = t1_x[warpPath[i].x];
            t1_y_dtw[i] = t1_y[warpPath[i].x];
            t1_z_dtw[i] = t1_z[warpPath[i].x];
            t1_a_dtw[i] = t1_a[warpPath[i].x];
            t1_b_dtw[i] = t1_b[warpPath[i].x];
            t1_c_dtw[i] = t1_c[warpPath[i].x];
            t1_d_dtw[i] = t1_d[warpPath[i].x];
            time1_dtw[i] = time1[warpPath[i].x];
    
            t2_x_dtw[i] = t2_x[warpPath[i].y];
            t2_y_dtw[i] = t2_y[warpPath[i].y];
            t2_z_dtw[i] = t2_z[warpPath[i].y];
            t2_a_dtw[i] = t2_a[warpPath[i].y];
            t2_b_dtw[i] = t2_b[warpPath[i].y];
            t2_c_dtw[i] = t2_c[warpPath[i].y];
            t2_d_dtw[i] = t2_d[warpPath[i].y];
            time2_dtw[i] = time2[warpPath[i].y];
    }

    /*double thresh = 0.00001;
    size_t start = 0;
    size_t end = 0;
    double dist; 
    for (size_t i = 1; i < warpPath.size(); i++){
        dist = normVector(t1_x_dtw[i],t1_y_dtw[i],t1_z_dtw[i])-normVector(t1_x_dtw[i-1],t1_y_dtw[i-1],t1_z_dtw[i-1]);
        ROS_INFO("Dist: %.10f", dist);
    	if(dist>thresh){
			start=i;
            ROS_INFO("Cutoff %d elements from the start.", i);
            break;
        }
	}
    
    for (size_t i = warpPath.size(); i<=0; i--){
        dist = normVector(t1_x_dtw[i],t1_y_dtw[i],t1_z_dtw[i])-normVector(t1_x_dtw[i-1],t1_y_dtw[i-1],t1_z_dtw[i-1]);
		if(dist>thresh){
			end=i;
            ROS_INFO("Cutoff %d elements from the end.",i);
            break;
        }    
    }
    
    trimVector(t1_x_dtw, start, end);
    trimVector(t1_y_dtw, start, end);
    trimVector(t1_z_dtw, start, end);
    trimVector(t1_a_dtw, start, end);
    trimVector(t1_b_dtw, start, end);
    trimVector(t1_c_dtw, start, end);
    trimVector(t1_d_dtw, start, end);
	
	trimVector(t2_x_dtw, start, end);
    trimVector(t2_y_dtw, start, end);
    trimVector(t2_z_dtw, start, end);
    trimVector(t2_a_dtw, start, end);
    trimVector(t2_b_dtw, start, end);
    trimVector(t2_c_dtw, start, end);
    trimVector(t2_d_dtw, start, end);*/

    ROS_INFO("t1_dtw size: %lu",t1_x_dtw.size());

    stringstream bagfile1_dtw;
    bagfile1_dtw << bagdir << bagfile << "_" << counter-1 << "_dtw" << ".bag";
    rosbag::Bag bag1_dtw(bagfile1_dtw.str(), rosbag::bagmode::Write);
    string bag_name_dtw1 = bag1_dtw.getFileName();
    ros::Time now = ros::Time::now();
    ros::Time bagtime = now;
    ros::Duration dt(time1[10]-time1[9]);
    int rosbag_ctr=0;
    geometry_msgs::Pose pose_dtw;   
    pose_dtw.orientation.x = t1_a[0];            
    pose_dtw.orientation.y = t1_b[0];            
    pose_dtw.orientation.z = t1_c[0];            
    pose_dtw.orientation.w = t1_d[0];
    
    for(size_t i=0; i<warpPath.size();i++){
    		bagtime = bagtime + dt;
            pose_dtw.position.x = t1_x_dtw[rosbag_ctr];
            pose_dtw.position.y = t1_y_dtw[rosbag_ctr];
            pose_dtw.position.z = t1_z_dtw[rosbag_ctr];
            bag1_dtw.write(bagtopic, bagtime, pose_dtw);

            rosbag_ctr++;          
    }

    ROS_INFO("Rosbagctr: %d", rosbag_ctr);
    bag1.close();
    bag1_dtw.close();
    stringstream bagfile2_dtw;
    bagfile2_dtw << bagdir << bagfile << "_" << counter << "_dtw" << ".bag";
    rosbag::Bag bag2_dtw(bagfile2_dtw.str(), rosbag::bagmode::Write);
    string bag_name_dtw2 = bag2_dtw.getFileName();

    bagtime = bagtime + ros::Duration(20.0);

    rosbag_ctr=0;
    for(size_t i=0;i<warpPath.size();i++){
            //TODO Check for msg_typename
    		bagtime = bagtime + dt;
            pose_dtw.position.x = t2_x_dtw[rosbag_ctr];
            pose_dtw.position.y = t2_y_dtw[rosbag_ctr];
            pose_dtw.position.z = t2_z_dtw[rosbag_ctr];
            // pose_dtw.orientation.x = t2_a_dtw[rosbag_ctr];            
            // pose_dtw.orientation.y = t2_b_dtw[rosbag_ctr];            
            // pose_dtw.orientation.z = t2_c_dtw[rosbag_ctr];            
            // pose_dtw.orientation.w = t2_d_dtw[rosbag_ctr];  
            
            bag2_dtw.write(bagtopic, bagtime, pose_dtw);
            
            rosbag_ctr++;
    }
    bag2.close();
    bag2_dtw.close();

    ROS_INFO("Bagsize_dtw 1: %lu",bag1_dtw.getSize());
    ROS_INFO("Bagsize_dtw 2: %lu",bag2_dtw.getSize());

    cmd::context ctx;
    ctx.stdin_behavior = cmd::inherit_stream();
    ctx.stdout_behavior = cmd::inherit_stream();
    ctx.environment = cmd::self::get_environment();
    cmd::child rosbag_shell = cmd::launch_shell("", ctx);

    //Delte unwarped bagfiles
    // cmd_stream << "rm " << bagdir << "/" << bag1.getFileName();
    // rosbag_cmd = cmd_stream.str();
    // rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
    // cmd_stream.clear();
    // cmd_stream << "rm " << bagdir << "/" << bag2.getFileName();
    // rosbag_cmd = cmd_stream.str();
    // rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
    // cmd_stream.clear();

    //Rename bagfiles

    stringstream cmd_stream;
    string rosbag_cmd;
    cmd_stream << "mv -f " << bag_name_dtw1 << " " << bag_name1;
    rosbag_cmd = cmd_stream.str();
    ROS_INFO("MV-Cmd: %s", rosbag_cmd.c_str());
    rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
    rosbag_shell.wait();
    stringstream cmd_stream2;   
    cmd_stream2 << "mv -f " << bag_name_dtw2 << " " << bag_name2;
    rosbag_cmd = cmd_stream2.str();
    ROS_INFO("MV-Cmd: %s", rosbag_cmd.c_str());
    rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
    rosbag_shell.wait();
    cmd_stream.clear();
}

void trimVector(std::vector<double> &vec, size_t start, size_t end){
	vec.erase(vec.begin(),vec.begin()+start);
    vec.erase(vec.end(), vec.end()-end);
}
double normVector(double x, double y, double z){
	return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}