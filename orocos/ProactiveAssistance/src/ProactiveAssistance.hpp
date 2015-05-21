#ifndef OROCOS_PROACTIVEASSISTANCE_HPP
#define OROCOS_PROACTIVEASSISTANCE_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <tf_conversions/tf_kdl.h>

#include <std_msgs/Empty.h>
#include <kuka_lwr_fri/friComm.h>
#include <lwr_fri/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

namespace iros {

using namespace RTT;
using namespace RTT::os;
using namespace std;

class ProactiveAssistance : public RTT::TaskContext{
  public:
    ProactiveAssistance(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    boost::array<float, 7> stiffness, damping;

  	InputPort<size_t> port_counter;

	OutputPort<geometry_msgs::Pose> port_cmd_position;
	InputPort<geometry_msgs::Pose> port_msr_position;
	OutputPort<motion_control_msgs::JointEfforts> port_cmd_addJntTrq;
	InputPort<lwr_fri::FriJointTorques> port_msr_extJntTrq;

	InputPort<std_msgs::Empty> port_start_assistance;
	InputPort<std_msgs::Empty> port_stop_assistance;
	OutputPort<std_msgs::Empty> port_isstarted_assistance;
	OutputPort<std_msgs::Empty> port_isstopped_assistance;

	InputPort<geometry_msgs::Vector3> port_force;
	InputPort<geometry_msgs::Vector3> port_torque;

	InputPort<geometry_msgs::Wrench> port_msr_kuka_wrench;
    	OutputPort<geometry_msgs::Wrench> port_cmd_kuka_wrench;

	geometry_msgs::Pose current_pose;
	geometry_msgs::Vector3 current_force;
	geometry_msgs::Vector3 current_torque;
	geometry_msgs::Wrench current_kuka_wrench;
	lwr_fri::FriJointTorques current_jnt_trq;

//	geometry_msgs::Twist stiffness;
//	geometry_msgs::Twist damping;

	TimeService::ticks start_ticks;
	TimeService::Seconds last_iteration_time;

	TaskContext* commander;

	OperationCaller<FRI_STATE(void)> getCurrentState;
	OperationCaller<FRI_CTRL(void)> getCurrentControlMode;
	OperationCaller<void(void)> switchToMonitorState;
	OperationCaller<void(void)> switchToCommandState;
	OperationCaller<void(void)> stopCommunication;
	OperationCaller<void(FRI_CTRL, FRI_STATE)> setControlMode;
	OperationCaller<void(geometry_msgs::Twist, geometry_msgs::Twist)> setCartesianImpedance;
	OperationCaller<void(boost::array<float, 7>, boost::array<float, 7>)> setJointImpedance;
	OperationCaller<void(bool)> activateGravityCompensation;
	OperationCaller<bool(boost::array<double, 7>, double)> moveToJointPosition;
	OperationCaller<bool(geometry_msgs::Pose, double)> moveToCartesianPosition;
	OperationCaller<bool(void)> stopMovements;
	OperationCaller<bool(void)> isMoving;
	OperationCaller<bool(boost::array<double, 7> jointPos, uint8_t speed)> jointPTPMotion;
	OperationCaller<bool(geometry_msgs::Pose CartPose, double speed)> CartesianLINMotion;
	OperationCaller<bool(geometry_msgs::Pose CartPose, uint8_t speed)> CartesianPTPMotion;
	OperationCaller<geometry_msgs::Quaternion(double, double, double)> getQuaternionFromRPY;  

  private:
	void startAssistance(RTT::base::PortInterface*);
	void stopAssistance(RTT::base::PortInterface*);
};
}
#endif
