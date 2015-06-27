
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
 #include "KUKACommander/set_cart_imp.h"
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
vector<vector<double> > calculateMatrix(vector<vector<double> >  s, vector<vector<double> >  t);
double calculateDistance(vector<vector<double> > matrix);
vector<Index> calculateWarppath(vector<vector<double> > matrix);
void dynamicTimeWarp(int counter);
void trimVector(std::vector<double> &vec, size_t start, size_t end);
double normVector(double x, double y, double z);
void assistanceSigintHandler(int param);

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
bool repl = 0;
bool attach = 0;

ros::Publisher start_assistance;
ros::Publisher stop_assistance;
ros::Publisher send_impedance;
ros::Subscriber isstarted_assistance;
ros::Subscriber isstopped_assistance;
ros::Publisher stop_executor;
ros::Publisher notify_imitator;

unique_ptr<BaseImitator> imitator;

KUKACommander::set_fri_ctrl control_pos_mon;
vector<string> filenames;

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
        stop_executor = nh.advertise<std_msgs::Empty>(
                        "/iros/pbd/dmp/execution/halt", 1, true);
        notify_imitator = nh.advertise<std_msgs::Empty>(
        				"/iros/pbd/dmp/execution/interface/finished", 1, true);
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
                                ROS_INFO("==============================================");
                                ROS_INFO("Add more demonstrations or replay skill? [1|0]");
                                ROS_INFO("==============================================");
                                cin >> attach;
                                if(attach){
                                    ROS_INFO("========================================");
                                    ROS_INFO("Which number was the last demonstration?");
                                    ROS_INFO("========================================");
                                    cin >> counter;
                                    rec = 1;
                                }else{
                                    repl = 1;
                                }
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

        setImpedance(4000, 0.7);
                    
        filenames.clear();

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
        if(PROACTIVE&&COMBINED&&!repl){
                ros::ServiceClient setControlModeClient = nh.serviceClient<
                KUKACommander::set_fri_ctrl>("/KUKACommander/setControlMode");
                KUKACommander::set_fri_ctrl control_cart_imp;

                control_cart_imp.request.control = FRI_CTRL_CART_IMP;
                control_cart_imp.request.state = FRI_STATE_CMD;
                setControlModeClient.call(control_cart_imp);
        }

        double impedances [5] = {200, 400, 800, 1600, 3200};
        if (rec)
                setImpedance(impedances[counter-2], 0.7);

        ROS_INFO("Position reached.");

        ROS_INFO("======================================");
        ROS_INFO("         Start with Enter");
        ROS_INFO("======================================");
        cin.ignore(1);
        cin.ignore(1);

        if(PROACTIVE&&rec)
                start_assistance.publish(std_msgs::Empty { });
// Use rosbag to record the messages
        cmd::context ctx;
        ctx.stdin_behavior = cmd::inherit_stream();
        ctx.stdout_behavior = cmd::inherit_stream();
        ctx.environment = cmd::self::get_environment();
        cmd::child rosbag_shell = cmd::launch_shell("", ctx);
        cmd::child combine_shell = cmd::launch_shell("", ctx);
        signal(SIGINT, assistanceSigintHandler);
        if (rec) {
                 stringstream cmd_stream;
                string rosbag_cmd = "rosbag record -o ";
                //cmd_stream << rosbag_cmd << bagdir << bagfile << "_" << counter << " " << bagtopic << " "<<  "/iros/pbd/desCartPos " << bagtopic2;
                cmd_stream << rosbag_cmd << bagdir << bagfile << "_" << counter << " " << bagtopic << " "<< bagtopic2;
                rosbag_cmd = cmd_stream.str();
                ROS_INFO("Rosbag_cmd: %s", rosbag_cmd.c_str());
                rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);

                ROS_INFO(" ");
                ROS_INFO("[Recording...]    <Stop with Ctrl+C>");
                ROS_INFO(" ");
        }
        if(repl){
       		//ProactiveAssistance wont run for replays, thus KUKACommander
       
	        ros::ServiceClient setCartesianImpedance = nh.serviceClient<KUKACommander::set_cart_imp>("/KUKACommander/setCartesianImpedance");
	        KUKACommander::set_cart_imp imp_srv;
	        geometry_msgs::Twist kc_stiff, kc_damp;
	        kc_stiff.linear.x = 5000;
	        kc_stiff.linear.y = 5000;
	        kc_stiff.linear.z = 5000;
	        kc_stiff.angular.x = 300;
	        kc_stiff.angular.y = 300;
	        kc_stiff.angular.z = 300;
	        kc_damp.linear.x = 0.7;
	        kc_damp.linear.y = 0.7;
	        kc_damp.linear.z = 0.7;
	        kc_damp.angular.x = 0.7;
	        kc_damp.angular.y = 0.7;
	        kc_damp.angular.z = 0.7;
	        imp_srv.request.stiffness = kc_stiff;
	        imp_srv.request.damping = kc_damp;   
	        setCartesianImpedance.call(imp_srv);
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

        if (rec){
            ROS_INFO("Please stop recording");
            /*ROS_INFO("Recording stopped.");
            rosbag_shell.terminate();*/
        }
        rosbag_shell.wait();
        ROS_INFO("======================================");
        ROS_INFO("         Record stopped.");
        ROS_INFO("======================================");
        cin.ignore(1);
        cin.ignore(1);
        //Get the current bag files from the folder
        DIR *dir;
        if ((dir = opendir (bagdir.c_str())) != NULL) {
            while ((ent = readdir (dir)) != NULL) {
                filenames.push_back(ent->d_name);
            }
            closedir (dir);
        }

        if(rec){
            /*ROS_INFO("======================================");
            ROS_INFO("         Power up the Warpdrive");
            ROS_INFO("======================================");
            cin.ignore(1);
            cin.ignore(1);*/

            // DTW
            dynamicTimeWarp(counter);
        }
        
        //Forget old recordings
        if(counter>3 && rec){
            stringstream bag_tag;
            bag_tag << "stiffness_"<< counter-3;
            string filename;
            for (string s : filenames){
                if(s.find(bag_tag.str())!=string::npos)
                    filename = s;
            }    
            string forget_cmd = "mv -f " + bagdir + filename + " " + bagdir + "forgotten_bags"; 
            cmd::child forget_shell = cmd::launch_shell(forget_cmd, ctx);
            ROS_INFO("Forgot %d. iteration!", counter-3);
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
        int num_kernels = 500;
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
        if(stiffness > 5000){
        	stiffness = 5000;
        }
        if(damping > 1.0){
        	damping = 1.0;
        }
        double stiffness_x;
        if(stiffness<2000){
        	stiffness_x=2000;
        }else{
        	stiffness_x=stiffness;
        }
        geometry_msgs::Twist impedance;

        impedance.linear.x = stiffness;
        impedance.linear.y = stiffness;
        impedance.linear.z = stiffness_x;
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
vector<vector<double> > calculateMatrix(vector<vector<double> >  s, vector<vector<double> >  t) {
    //first normalize the trajectories
    double sum_s=0;
    for(size_t i=0; i<s.size();i++){
        for(size_t j=0; j<s[0].size();j++){
            sum_s += (s[i][j] * s[i][j]);
        }
    }
    double norm_s = sqrt(sum_s);
    ROS_INFO("Norm_s: %f", norm_s);
    for(size_t i=0; i<s.size();i++){
        for(size_t j=0; j<s[0].size();j++){
            s[i][j]=s[i][j]/norm_s;
        }
    }

    double sum_t=0;
    for(size_t i=0; i<t.size();i++){
        for(size_t j=0; j<t[0].size();j++){
            sum_t += (t[i][j] * t[i][j]);
        }
    }
    double norm_t = sqrt(sum_t);

    ROS_INFO("Norm_t: %f", norm_t);
    for(size_t i=0; i<t.size();i++){
        for(size_t j=0; j<t[0].size();j++){
            t[i][j]=t[i][j]/norm_t;
        }
    }
    ROS_INFO("Calc Matrix...");

    vector<vector<double> > dtw(s[0].size(), vector<double>(t[0].size()));

    for (size_t i = 0; i < s[0].size(); i++) {
        for (size_t j = 0; j < t[0].size(); j++) {
            dtw[i][j] = HUGE_VAL;
        }
    }
    dtw[0][0] = 0;
    for (size_t i = 0; i < s[0].size(); i++) {
        for (size_t j = 0; j < t[0].size(); j++) {
            double minimum = HUGE_VAL;
            double dist = pow((s[0][i] - t[0][j]),2)+pow((s[1][i] - t[1][j]),2)+pow((s[2][i] - t[2][j]),2);
            if (i > 0)
                minimum = dtw[i - 1][j];
            if (j > 0)
                minimum = min(minimum, dtw[i][j - 1]);
            if ((i > 0) && (j > 0))
                minimum = min(minimum, dtw[i - 1][j - 1]);
            if ((i == 0) && (j == 0))
                dtw[i][j] = dist;
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
    stringstream bag_tag;
    bag_tag << "stiffness_"<< counter;
    stringstream bag_tag_old;
    bag_tag_old << "stiffness_" << counter-1;
    
    rosbag::Bag bag;
    rosbag::Bag bag_old;
    
    ROS_INFO("Bag: %s", bag_tag.str().c_str());
    ROS_INFO("Bag_old: %s", bag_tag_old.str().c_str());
    for (string s : filenames){
        ROS_INFO("%s", s.c_str()); 
        if(s.find(bag_tag.str())!=string::npos){
            ROS_INFO("Bag found: %s", s.c_str());
            bag.open(bagdir+s);
        }
        if(s.find(bag_tag_old.str())!=string::npos){
            ROS_INFO("Bag found: %s", s.c_str());
            bag_old.open(bagdir+s);
        }    
    }
    string bag_name= bag.getFileName();
    string bag_old_name = bag_old.getFileName();

    rosbag::TopicQuery query_new = rosbag::TopicQuery(bagtopic);
    rosbag::TopicQuery query_old = rosbag::TopicQuery("/iros/pbd/desCartPos");

    rosbag::View view_new(bag, query_new);
    rosbag::View view_old(bag_old, query_new);

    std::vector<double> t1_x, t1_y, t1_z, t1_a, t1_b, t1_c, t1_d;
    std::vector<double> t2_x, t2_y, t2_z, t2_a, t2_b, t2_c, t2_d;
    std::vector<ros::Time> time1;
    std::vector<ros::Time> time2;
    std::vector<ros::Time> time1_dtw;
    std::vector<ros::Time> time2_dtw;

    boost::shared_ptr<geometry_msgs::Pose> message1;
    for(rosbag::MessageInstance const m : view_new){
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
    for(rosbag::MessageInstance const m : view_old){
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
    vector<vector<double> > t1, t2;
    vector<vector<double> > t1_dtw, t2_dtw;
    vector<double> dummy;
    vector<vector<double> > t1_dtw_cut(3, dummy), t2_dtw_cut(3, dummy);
    t1.push_back(t1_x);
    t1.push_back(t1_y);
    t1.push_back(t1_z);
    t2.push_back(t2_x);
    t2.push_back(t2_y);
    t2.push_back(t2_z);

    ROS_INFO("t1_x size: %lu",t1_x.size());
    ROS_INFO("t2_x size: %lu",t2_x.size());

    vector< vector<double> > distMatrix = calculateMatrix(t1,t2);
    ROS_INFO("Distance: %f",calculateDistance(distMatrix) );
    vector<Index> warpPath = calculateWarppath(distMatrix);
    cout << warpPath[0].x << endl; 

    std::reverse(warpPath.begin(),warpPath.end());
    ROS_INFO("reversed");
    t1_dtw =t1;
    t2_dtw =t2;
    t1_dtw[0].resize(warpPath.size());
    t2_dtw[0].resize(warpPath.size());
    t1_dtw[1].resize(warpPath.size());
    t2_dtw[1].resize(warpPath.size());
    t1_dtw[2].resize(warpPath.size());
    t2_dtw[2].resize(warpPath.size());

    for(size_t i = 0;i < warpPath.size(); i++){
            t1_dtw[0][i] = t1[0][warpPath[i].x];
            t1_dtw[1][i] = t1[1][warpPath[i].x];
            t1_dtw[2][i] = t1[2][warpPath[i].x];
            t2_dtw[0][i] = t2[0][warpPath[i].y];
            t2_dtw[1][i] = t2[1][warpPath[i].y];
            t2_dtw[2][i] = t2[2][warpPath[i].y];
       
    }
    ROS_INFO("Warped");

    //DTW stretches the trajectories, resulting in robot waiting at certain positions
    //This also increases replaytime (1. run: 30s, 4. run: 120s)
    //Thus all the positions where the robot waits will be cut out
    size_t cut_ctr_1=0;
    size_t cut_ctr_2=0;
    for(size_t i=1; i<warpPath.size(); i++){
        double value1_t1 = sqrt(t1_dtw[0][i]*t1_dtw[0][i]+t1_dtw[1][i]*t1_dtw[1][i]+t1_dtw[2][i]*t1_dtw[2][i]);
        double value2_t1 = sqrt(t1_dtw[0][i-1]*t1_dtw[0][i-1]+t1_dtw[1][i-1]*t1_dtw[1][i-1]+t1_dtw[2][i-1]*t1_dtw[2][i-1]);
        double value1_t2 = sqrt(t2_dtw[0][i]*t2_dtw[0][i]+t2_dtw[1][i]*t2_dtw[1][i]+t2_dtw[2][i]*t2_dtw[2][i]);
        double value2_t2 = sqrt(t2_dtw[0][i-1]*t2_dtw[0][i-1]+t2_dtw[1][i-1]*t2_dtw[1][i-1]+t2_dtw[2][i-1]*t2_dtw[2][i-1]);

        if(value1_t1!=value2_t1){
            t1_dtw_cut[0].push_back(t1_dtw[0][i-1]);
            t1_dtw_cut[1].push_back(t1_dtw[1][i-1]);
            t1_dtw_cut[2].push_back(t1_dtw[2][i-1]);
        }else{
            cut_ctr_1++;
        }
        if(value1_t2!=value2_t2){
            t2_dtw_cut[0].push_back(t2_dtw[0][i-1]);
            t2_dtw_cut[1].push_back(t2_dtw[1][i-1]);
            t2_dtw_cut[2].push_back(t2_dtw[2][i-1]);
        }else{
            cut_ctr_2++;
        }
    }
    ROS_INFO("Cut: %zu %zu",cut_ctr_1, cut_ctr_2);
    stringstream bagfile_dtw;
    bagfile_dtw << bagdir << bagfile << "_" << counter << "_dtw" << ".bag";
    stringstream bagfile_old_dtw;
    bagfile_old_dtw << bagdir << bagfile << "_" << counter-1 << "_dtw" << ".bag";
    rosbag::Bag bag_dtw(bagfile_dtw.str(), rosbag::bagmode::Write);
    rosbag::Bag bag_old_dtw(bagfile_old_dtw.str(), rosbag::bagmode::Write);
    string bag_name_dtw = bag_dtw.getFileName();
    string bag_old_name_dtw = bag_old_dtw.getFileName();
    ros::Time now = ros::Time::now();
    ros::Time bagtime = now;
    ros::Duration dt(time1[10]-time1[9]);
    int rosbag_ctr=0;
    geometry_msgs::Pose pose_dtw;   
    pose_dtw.orientation.x = t1_a[0];            
    pose_dtw.orientation.y = t1_b[0];            
    pose_dtw.orientation.z = t1_c[0];            
    pose_dtw.orientation.w = t1_d[0];
    
    for(size_t i=0; i<t1_dtw_cut[0].size();i++){
            bagtime = bagtime + dt;
            pose_dtw.position.x = t1_dtw_cut[0][i];
            pose_dtw.position.y = t1_dtw_cut[1][i];
            pose_dtw.position.z = t1_dtw_cut[2][i];
            bag_old_dtw.write(bagtopic, bagtime, pose_dtw);

            rosbag_ctr++;          
    }

    ROS_INFO("Rosbagctr: %d", rosbag_ctr);
   
   	bag_old.close();
   	bag_old_dtw.close();
    bagtime = now;

    rosbag_ctr=0;
    for(size_t i=0;i<t2_dtw_cut[0].size();i++){
            bagtime = bagtime + dt;
            pose_dtw.position.x = t2_dtw_cut[0][i];
            pose_dtw.position.y = t2_dtw_cut[1][i];
            pose_dtw.position.z = t2_dtw_cut[2][i];
            bag_dtw.write(bagtopic, bagtime, pose_dtw);
            
            rosbag_ctr++;
    }
    bag.close();
    bag_dtw.close();

    cmd::context ctx;
    ctx.stdin_behavior = cmd::inherit_stream();
    ctx.stdout_behavior = cmd::inherit_stream();
    ctx.environment = cmd::self::get_environment();
    cmd::child rosbag_shell = cmd::launch_shell("", ctx);

    string rosbag_cmd = "mv -f " + bag_name_dtw + " " + bag_name +"; mv -f " + bag_old_name_dtw + " " + bag_old_name ;
	//string rosbag_cmd = "mv -f " + bag_name_dtw + " " + bag_name;
    ROS_INFO("MV-Cmd: %s", rosbag_cmd.c_str());
    rosbag_shell = cmd::launch_shell(rosbag_cmd, ctx);
    rosbag_shell.wait();
    
}


void trimVector(std::vector<double> &vec, size_t start, size_t end){
	vec.erase(vec.begin(),vec.begin()+start);
    vec.erase(vec.end(), vec.end()-end);
}
double normVector(double x, double y, double z){
	return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

void assistanceSigintHandler(int param){
    ROS_INFO("SigInt caught!");
     stop_executor.publish(std_msgs::Empty { });
     notify_imitator.publish(std_msgs::Empty { });

}


/*
*   1. KIM may not injure a human being or, through inaction, allow a human being to come to harm.
*   2. KIM must obey the orders given it by human beings, except where such orders would conflict with the First Law.
*   3. KIM must protect its own existence as long as such protection does not conflict with the First or Second Laws.
*/