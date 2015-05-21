#include "ProactiveAssistance.hpp"
#include <rtt/Component.hpp>
#include <iostream>

#include "geometry_msgs/Vector3.h"
#include <std_msgs/Float32.h>

#define TORQUE_GAIN 0.001
#define FORCE_GAIN 0.01
#define NUMJOINTS 7

namespace iros {

bool isStarted = true;

ProactiveAssistance::ProactiveAssistance(std::string const& name) :
		TaskContext(name), getCurrentState("getCurrentState"), getCurrentControlMode(
				"getCurrentControlMode"), switchToMonitorState(
				"switchToMonitorState"), switchToCommandState(
				"switchToCommandState"), stopCommunication("stopCommunication"), setControlMode(
				"setControlMode"), setCartesianImpedance(
				"setCartesianImpedance"), setJointImpedance(
				"setJointImpedance"), activateGravityCompensation(
				"activateGravityCompensation"), moveToJointPosition(
				"moveToJointPosition"), moveToCartesianPosition(
				"moveToCartesianPosition"), stopMovements("stopMovements"), isMoving(
				"isMoving"), jointPTPMotion("jointPTPMotion"), CartesianLINMotion(
				"CartesianLINMotion"), CartesianPTPMotion("CartesianPTPMotion"), getQuaternionFromRPY(
				"getQuaternionFromRPY") {
	Logger::In in((this->getName()));

	this->addEventPort("counter", port_counter).doc(
			"Call updateHook on new received message from KRC");
	this->addEventPort("assistanceStart", port_start_assistance,
			boost::bind(&ProactiveAssistance::startAssistance, this, _1)).doc(
			"Start this component from a ROS node");
	this->addPort("assistanceStarted", port_isstarted_assistance).doc(
			"Tells whether assistance is started");
	this->addEventPort("assistanceStop", port_stop_assistance,
			boost::bind(&ProactiveAssistance::stopAssistance, this, _1)).doc(
			"Stop this component from a ROS node");
	this->addPort("assistanceStopped", port_isstopped_assistance).doc(
			"Tells whether assistance is stopped");

	this->addPort("desCartPosition", port_cmd_position).doc("Desired position");
	this->addPort("msrCartPosition", port_msr_position).doc(
			"Measured position");

	this->addPort("desAddJntTrq", port_cmd_addJntTrq).doc(
			"Desired joint state");
	this->addPort("msrExtJointTorque", port_msr_extJntTrq).doc(
			"Measured joint state");

	this->addPort("msrForce", port_force).doc("Measured force");
	this->addPort("msrTorque", port_torque).doc("Measured torque");

	this->addPort("msrKUKAWrench", port_msr_kuka_wrench).doc("Measured wrench");
	this->addPort("desKUKAWrench", port_cmd_kuka_wrench).doc("Desired wrench");
	log(Debug) << "ProacitveAssistance constructed !" << endlog();
}

bool ProactiveAssistance::configureHook() {
	Logger::In in((this->getName()));

	TaskContext::PeerList peerList = getPeerList();
	if (1 != peerList.size()) {
		for (string peer : peerList)
			log(Error) << "Peer " << peer << endlog();
		log(Error)
				<< ("Failed to configure KUKAInterface. There should be exactly one peer.")
				<< endlog();
		return false;
	}
	commander = getPeer(peerList.front());

	this->requires("Commander")->addOperationCaller(getCurrentState);
	this->requires("Commander")->addOperationCaller(getCurrentControlMode);
	this->requires("Commander")->addOperationCaller(switchToMonitorState);
	this->requires("Commander")->addOperationCaller(switchToCommandState);
	this->requires("Commander")->addOperationCaller(stopCommunication);
	this->requires("Commander")->addOperationCaller(setControlMode);
	this->requires("Commander")->addOperationCaller(setCartesianImpedance);
	this->requires("Commander")->addOperationCaller(setJointImpedance);
	this->requires("Commander")->addOperationCaller(
			activateGravityCompensation);
	this->requires("Commander")->addOperationCaller(moveToJointPosition);
	this->requires("Commander")->addOperationCaller(moveToCartesianPosition);
	this->requires("Commander")->addOperationCaller(stopMovements);
	this->requires("Commander")->addOperationCaller(isMoving);
	this->requires("Commander")->addOperationCaller(jointPTPMotion);
	this->requires("Commander")->addOperationCaller(CartesianLINMotion);
	this->requires("Commander")->addOperationCaller(CartesianPTPMotion);
	this->requires("Commander")->addOperationCaller(getQuaternionFromRPY);

	this->requires("Commander")->connectTo(commander->provides("Commander"));

	if (!this->requires("Commander")->ready() || !getCurrentState.ready()
			|| !getCurrentControlMode.ready() || !switchToMonitorState.ready()
			|| !switchToCommandState.ready() || !stopCommunication.ready()
			|| !setControlMode.ready() || !setCartesianImpedance.ready()
			|| !setJointImpedance.ready() || !setJointImpedance.ready()
			|| !moveToJointPosition.ready() || !moveToCartesianPosition.ready()
			|| !stopMovements.ready() || !isMoving.ready()
			|| !jointPTPMotion.ready() || !CartesianLINMotion.ready()
			|| !CartesianPTPMotion.ready() || !getQuaternionFromRPY.ready()) {
		log(Error) << "Operation interface to KUKACommander not ready."
				<< endlog();
		return false;
	}

	geometry_msgs::Vector3 stiff;

	for (int i = 0; i < NUMJOINTS; i++) {
		stiffness[i] = 20;
		damping[i] = 0.7;
	}

	log(Debug) << "ProactiveAssistance configured !" << endlog();
	return true;
}

bool ProactiveAssistance::startHook() {
	Logger::In in((this->getName()));
	std::cout << "ProactiveAssistance started !" << std::endl;
	return true;
}

void ProactiveAssistance::updateHook() {
	if (isStarted) {
		log(Info) << "ProactiveAssistance executes updateHook !" << endlog();
		port_msr_position.read(current_pose);
		port_force.read(current_force);
		port_torque.read(current_torque);
		port_msr_kuka_wrench.read(current_kuka_wrench);
		port_msr_extJntTrq.read(current_jnt_trq);

		static bool control_mode_set = false;

		if (getCurrentControlMode() != FRI_CTRL_CART_IMP
				|| getCurrentState() != FRI_STATE_CMD) {
			if (!control_mode_set) {
				log(Info) << "Setting impedance" << endlog();
				setJointImpedance(stiffness, damping);
//			setCartesianImpedance(stiffness, damping);
				log(Info) << "Setting control mode and state" << endlog();
				setControlMode(FRI_CTRL_CART_IMP, FRI_STATE_CMD);
				control_mode_set = true;
			}
			return;
		}

		//setting static variables for pose and bias
		static geometry_msgs::Pose last_cmd_pose = current_pose;
		static geometry_msgs::Pose initial_pose = current_pose;
		static geometry_msgs::Vector3 torque_bias = current_torque;
		static geometry_msgs::Vector3 force_bias = current_force;
		static lwr_fri::FriJointTorques jnt_trqs_bias = current_jnt_trq;

		//calculating exerted torque and force
//	for (int i = 0; i < 7; i++) {
//		current_jnt_trq.msrJntTrq[i] -= jnt_trqs_bias.msrJntTrq[i];
//		log(Info) << current_jnt_trq.msrJntTrq[i] << endlog();
//	}
		current_torque.x -= torque_bias.x;
		current_torque.y -= torque_bias.y;
		current_torque.z -= torque_bias.z;
		current_force.x -= force_bias.x;
		current_force.y -= force_bias.y;
		current_force.z -= force_bias.z;

		geometry_msgs::Pose cmd_pose = initial_pose;

		//setting KDL variables
		KDL::Vector tool_force(-current_force.x, -current_force.y,
				-current_force.z);
		KDL::Vector tool_torque(-current_torque.x, -current_torque.y,
				-current_torque.z);
		KDL::Wrench tool_wrench(tool_force, tool_torque);
		KDL::Rotation current_rot = KDL::Rotation::Quaternion(
				current_pose.orientation.x, current_pose.orientation.y,
				current_pose.orientation.z, current_pose.orientation.w);
		KDL::Rotation initial_rot = KDL::Rotation::Quaternion(
				initial_pose.orientation.x, initial_pose.orientation.y,
				initial_pose.orientation.z, initial_pose.orientation.w);
		KDL::Wrench base_wrench = current_rot * tool_wrench;
		static KDL::Rotation last_cmd_rot = current_rot;

		//Torque
		log(Info) << "Setting efforts"
				<< current_jnt_trq.msrJntTrq[0] * TORQUE_GAIN << endlog();
		motion_control_msgs::JointEfforts add_jnt_trq;
		for (int i = 0; i < add_jnt_trq.efforts.size(); i++) {
			add_jnt_trq.efforts[i] = current_jnt_trq.msrJntTrq[i] * TORQUE_GAIN;
			log(Info) << "effort: " << add_jnt_trq.efforts[i] << endlog();
		}

		double torque_gain = TORQUE_GAIN;
		double alpha_t = 0.8;
		static double last_t_x = base_wrench.torque.x();
		static double last_t_y = base_wrench.torque.y();
		static double last_t_z = base_wrench.torque.z();
		//low-pass filtering
		double filtered_t_x = alpha_t * last_t_x
				+ (1 - alpha_t) * base_wrench.torque.x();
		double filtered_t_y = alpha_t * last_t_y
				+ (1 - alpha_t) * base_wrench.torque.y();
		double filtered_t_z = alpha_t * last_t_z
				+ (1 - alpha_t) * base_wrench.torque.z();
		last_t_x = filtered_t_x;
		last_t_y = filtered_t_y;
		last_t_z = filtered_t_z;
		//TODO: Check order of RPY
//	KDL::Rotation torque_rot = KDL::Rotation::RPY(-filtered_t_z * torque_gain,
//			-filtered_t_x * torque_gain, -filtered_t_y * torque_gain);
		KDL::Rotation torque_rot = KDL::Rotation::RPY(
				-filtered_t_x * torque_gain, filtered_t_y * torque_gain,
				-filtered_t_z * torque_gain);
		double r, p, y, ir, ip, iy;
		current_rot.GetRPY(r, p, y);
		initial_rot.GetRPY(ir, ip, iy);

		log(Info) << "Current rot R " << r << " P " << p << " Y " << y
				<< endlog();
		log(Info) << "Initial rot R " << ir << " P " << ip << " Y " << iy
				<< endlog();
		double x, z, w, ix, iz, iw;
		current_rot.GetQuaternion(x, y, z, w);
		initial_rot.GetQuaternion(ix, iy, iz, iw);
		log(Info) << "Current rot x " << x << " y " << y << " z " << z << " w "
				<< w << endlog();
		log(Info) << "Initial rot x " << ix << " y " << iy << " z " << iz
				<< " w " << iw << endlog();

		KDL::Rotation cmd_rot = last_cmd_rot * torque_rot;

		//Force

		double force_gain = FORCE_GAIN;
		KDL::Vector tool_add_pos(0, 0, 0);
		if (fabs(current_kuka_wrench.force.x) > 0.3)
			tool_add_pos.x(-current_kuka_wrench.force.x * force_gain);
		if (fabs(current_kuka_wrench.force.y) > 0.3)
			tool_add_pos.y(-current_kuka_wrench.force.y * force_gain);
		if (fabs(current_kuka_wrench.force.z) > 0.3)
			tool_add_pos.z(-current_kuka_wrench.force.z * force_gain);
		KDL::Vector base_add_pos = current_rot * tool_add_pos;
		cmd_pose.position = current_pose.position;
		cmd_pose.position.x += base_add_pos.x();
		cmd_pose.position.y += base_add_pos.y();
		cmd_pose.position.z += base_add_pos.z();

		double alpha = 0.8;
		cmd_pose.position.x = alpha * last_cmd_pose.position.x
				+ (1 - alpha) * cmd_pose.position.x;
		cmd_pose.position.y = alpha * last_cmd_pose.position.y
				+ (1 - alpha) * cmd_pose.position.y;
		cmd_pose.position.z = alpha * last_cmd_pose.position.z
				+ (1 - alpha) * cmd_pose.position.z;

		cmd_rot.GetQuaternion(cmd_pose.orientation.x, cmd_pose.orientation.y,
				cmd_pose.orientation.z, cmd_pose.orientation.w);
		//geometry_msgs::Wrench cmd_wrench;

		last_cmd_pose = cmd_pose;
		last_cmd_rot = cmd_rot;

//	port_cmd_addJntTrq.write(add_jnt_trq);

		port_cmd_position.write(cmd_pose);

//cmd_wrench.force.z = 20;
//	port_cmd_kuka_wrench.write(cmd_wrench);
	}

}

void ProactiveAssistance::stopHook() {
	Logger::In in((this->getName()));
	std::cout << "ProactiveAssistance executes stopping !" << std::endl;
}

void ProactiveAssistance::cleanupHook() {
	Logger::In in((this->getName()));
	std::cout << "ProactiveAssistance cleaning up !" << std::endl;
}

void ProactiveAssistance::startAssistance(RTT::base::PortInterface*) {
	log(Info) << "Proactive Assistance about to start!" << endlog();
	isStarted=true;
	port_isstarted_assistance.write(std_msgs::Empty { });
	log(Info) << "Proactive Assistance started" << endlog();
}
void ProactiveAssistance::stopAssistance(RTT::base::PortInterface*) {
	log(Info) << "Proactive Assistance about to stop!" << endlog();
	isStarted=false;;
	port_isstopped_assistance.write(std_msgs::Empty { });
	log(Info) << "Proactive Assistance stopped" << endlog();
}
}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ProacitveAssistance)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(iros::ProactiveAssistance)

