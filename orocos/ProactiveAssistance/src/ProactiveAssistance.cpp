#include "ProactiveAssistance.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <math.h>

#include "geometry_msgs/Vector3.h"
#include <std_msgs/Float32.h>

#define FORCE_FACTOR -1.5
#define TORQUE_FACTOR 0.0
#define FORCE_LIMIT 100.0
#define TORQUE_LIMIT 2.5
#define NUMJOINTS 7

#define CARTESIAN 1

namespace iros {

bool isStarted = false;
geometry_msgs::Pose last_pose_in_m;
std_msgs::Float32 last_timestamp;
KDL::Vector last_velocity;
int imp_update_counter=0;
double current_stiffness_z=1;
double in_contact_thresh =-0.5;

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

	this->addPort("timestamp", port_timestamp).doc("Current timestamp from KRC");

	this->addPort("desCartPos", port_des_position).doc("Desired position");
	this->addPort("msrCartPosition", port_msr_position).doc(
			"Measured position");

	this->addPort("desAddJntTrq", port_cmd_addJntTrq).doc(
			"Desired joint state");
	this->addPort("msrExtJointTorque", port_msr_extJntTrq).doc(
			"Measured joint state");

	this->addPort("msrForce", port_force).doc("Measured force");
	this->addPort("msrTorque", port_torque).doc("Measured torque");

	this->addPort("impedance", port_impedance).doc("Impedance");

	this->addPort("msrKUKAWrench", port_msr_kuka_wrench).doc("Measured wrench");
	this->addPort("desKUKAWrench", port_cmd_kuka_wrench).doc("Desired wrench");
	log(Debug) << "ProactiveAssistance constructed !" << endlog();
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

		Logger::In in((this->getName()));
		//Reading out the ports
		port_msr_position.read(current_pose);
		port_force.read(current_force);
		port_torque.read(current_torque);
		port_msr_kuka_wrench.read(current_kuka_wrench);
		port_msr_extJntTrq.read(current_jnt_trq);
		port_timestamp.read(current_timestamp);
		port_impedance.read(current_impedance);
		imp_update_counter++;	
	
		//Setting the damping
		damping.linear.x = 0.7;
		damping.linear.y = 0.7;
		damping.linear.z = 0.7;
		damping.angular.x = 0.7;
		damping.angular.y = 0.7;
		damping.angular.z = 0.7;

		static geometry_msgs::Pose init_pose = current_pose;

		if(CARTESIAN){

					
			static bool control_mode_set = false;

			//Set control and impedance
			if (getCurrentControlMode() != FRI_CTRL_CART_IMP
					|| getCurrentState() != FRI_STATE_CMD) {
				if (!control_mode_set) {
					log(Info) << "Setting control mode and state" << endlog();
					setControlMode(FRI_CTRL_CART_IMP, FRI_STATE_CMD);
					//switchToCommandState();
					setCartesianImpedance(stiffness, damping);
					control_mode_set = true;
				}
				return;
			}
				// setCartesianImpedance(stiffness, damping);

			KDL::Vector current_acceleration;
			if(imp_update_counter%100==0){
				float delta_t = current_timestamp.data-last_timestamp.data;
				geometry_msgs::Pose current_pose_in_m;

				current_pose_in_m.position.x = current_pose.position.x ;
				current_pose_in_m.position.y = current_pose.position.y ;
				current_pose_in_m.position.z = current_pose.position.z ;

				KDL::Vector current_velocity((current_pose_in_m.position.x-last_pose_in_m.position.x)/delta_t,
					(current_pose_in_m.position.y-last_pose_in_m.position.y)/delta_t,
					(current_pose_in_m.position.z-last_pose_in_m.position.z)/delta_t);

				KDL::Vector acceleration((current_velocity.x()-last_velocity.x())/delta_t, 
					(current_velocity.y()-last_velocity.y())/delta_t, 
					(current_velocity.z()-last_velocity.z())/delta_t);

				current_acceleration=acceleration;

				double vel = sqrt(pow(current_velocity.x(),2)+pow(current_velocity.y(),2)+pow(current_velocity.z(),2));
				double acc = sqrt(pow(current_acceleration.x(),2)+pow(current_acceleration.y(),2)+pow(current_acceleration.z(),2));
				double wrist_torque = sqrt(pow(current_kuka_wrench.torque.x,2)+pow(current_kuka_wrench.torque.y,2)+pow(current_kuka_wrench.torque.z,2));
				double tool_torque = sqrt(pow(current_torque.x,2)+pow(current_torque.y,2)+pow(current_torque.z,2)); 

				last_pose_in_m = current_pose_in_m;
				last_velocity = current_velocity;
				last_timestamp.data=current_timestamp.data;

				// log(Info) << "timestamp: " << current_timestamp.data << endlog(); 
				// log(Info) << "Current velocity: " << vel << " Current acceleration: " << acc << "Delta_t: "<< delta_t << endlog();
				// log(Info) << "last_pose: " << last_pose_in_m << endlog();
				// log(Info) << "last_velocity: " << last_velocity.x() << ", "<< last_velocity.y() << ", "<< last_velocity.z() << ", "<< endlog();
			}

			//setting the force amplification
			double force_factor_x = FORCE_FACTOR;//* 10 * current_acceleration.x();
			double force_factor_y = FORCE_FACTOR;//* 10 * current_acceleration.y();
			double force_factor_z = FORCE_FACTOR;//* 10 * current_acceleration.z();
			double torque_factor_x = TORQUE_FACTOR;
			double torque_factor_y = TORQUE_FACTOR;
			double torque_factor_z = TORQUE_FACTOR;

			//biases get set at first run
	 		static geometry_msgs::Vector3 wrist_force_bias = current_kuka_wrench.force;
			static geometry_msgs::Vector3 wrist_torque_bias = current_kuka_wrench.torque;
			static geometry_msgs::Vector3 tool_force_bias = current_force;
			static geometry_msgs::Vector3 tool_torque_bias = current_torque;

			//removing bias from FT sensor
			current_force.x -= tool_force_bias.x;
			current_force.y -= tool_force_bias.y;
			current_force.z -= tool_force_bias.z;
			current_torque.x -= tool_torque_bias.x;
			current_torque.y -= tool_torque_bias.y;
			current_torque.z -= tool_torque_bias.z;

			geometry_msgs::Wrench addWrench;
			//Low pass filter for force reading of FT sensor
			double alpha = 0.8;
			static double last_f_z = current_force.z;
			double filtered_f_z = alpha * last_f_z
					+ (1 - alpha) * current_force.z;

			last_f_z = filtered_f_z;
			if(imp_update_counter%100==0)
			log(Info) << "filtered_f_z: " << filtered_f_z << endlog();		
			//C -140 - 140


			//things to do if in contact
			if(filtered_f_z<in_contact_thresh){
				//the in_contact_thresh is higher if in contact to avoid jumping
				in_contact_thresh = -0.2;
				if(imp_update_counter%100==0)
				log(Info) << "In Contact!" << endlog();
				force_factor_z = 0;

				addWrench.force.x = current_kuka_wrench.force.x * force_factor_x;
				addWrench.force.y = current_kuka_wrench.force.y * force_factor_y;
				addWrench.force.z = 0;
				addWrench.torque.x = 0;
				addWrench.torque.y = 0;
				addWrench.torque.z = 0;

				//High stiffness in z direction to avoid jumping
				//High stiffness for angular movement to avoid turning of the tool
				stiffness.linear.x = current_impedance.linear.x;
				stiffness.linear.y = current_impedance.linear.y;
				stiffness.linear.z = 4000;
				stiffness.angular.x = 300;
				stiffness.angular.y = 300;
				stiffness.angular.z = 300;
			}else{
				in_contact_thresh = -1.0;
				stiffness.linear.x = current_impedance.linear.x;
				stiffness.linear.y = current_impedance.linear.y;
				stiffness.linear.z = current_impedance.linear.z;
				stiffness.angular.x = 300;
				stiffness.angular.y = 300;
				stiffness.angular.z = 300;

				addWrench.force.x = current_kuka_wrench.force.x * force_factor_x;
				addWrench.force.y = current_kuka_wrench.force.y * force_factor_y;
				addWrench.force.z = current_kuka_wrench.force.z * force_factor_z;
				addWrench.torque.x = current_kuka_wrench.torque.x * torque_factor_x;
				addWrench.torque.y = current_kuka_wrench.torque.y * torque_factor_y;
				addWrench.torque.z = current_kuka_wrench.torque.z * torque_factor_z;
			}

			if(stiffness.linear.z!=current_stiffness_z){
				setCartesianImpedance(stiffness, damping);
				log(Info) << "************* Changed Impedance *************" << endlog();
				log(Info) << "Stiffness: " << stiffness << endlog();
				log(Info) << "filtered_f_z: " << filtered_f_z << endlog();	
				current_stiffness_z=stiffness.linear.z;
			}
			
			

			//Safety measures to prevent robot from achieving world dominance
			if(abs(addWrench.force.x) > FORCE_LIMIT){
				addWrench.force.x=0.0;
				log(Warning) << "FORCE_LIMIT exceeded!" << endlog();
			}

			if(abs(addWrench.force.y) > FORCE_LIMIT){
				addWrench.force.y=0.0;
				log(Warning) << "FORCE_LIMIT exceeded!" << endlog();
			}

			if(abs(addWrench.force.z) > FORCE_LIMIT){
				addWrench.force.z = 0.0;
				log(Warning) << "FORCE_LIMIT exceeded!" << endlog();
			}

			if(abs(addWrench.torque.x) > TORQUE_LIMIT){
				log(Warning) << "TORQUE_LIMIT exceeded!" << endlog();
				addWrench.torque.x = 0.0;
			}
			if(abs(addWrench.torque.y) > TORQUE_LIMIT){
				log(Warning) << "TORQUE_LIMIT exceeded!" << endlog();
				addWrench.torque.y = 0.0;
			}
			if(abs(addWrench.torque.z) > TORQUE_LIMIT){
				log(Warning) << "TORQUE_LIMIT exceeded!" << endlog();
				addWrench.torque.z = 0.0;
			}

			geometry_msgs::Pose cmd_pose;
			//Setting current pose as setpoint for cartesian_imp_cntrl. 
			//IMPORTANT: Removing this will cause the robot to move directly and fast 
			//to initial position (1. setpoint) if contact is made (due to the high z-stiffness).
  			port_des_position.write(current_pose);
 
			port_cmd_kuka_wrench.write(addWrench);


		}else{

			static bool control_mode_set = false;

			if (getCurrentControlMode() != FRI_CTRL_JNT_IMP
					|| getCurrentState() != FRI_STATE_CMD) {
				if (!control_mode_set) {
					if(imp_update_counter%100==0)
					log(Info) << "Setting control mode and state" << endlog();
					setControlMode(FRI_CTRL_JNT_IMP, FRI_STATE_CMD);
					control_mode_set = true;
				}
				return;
			}
			boost::array<float, 7> jnt_stiffness, jnt_damping;
			for (int i=0;i<NUMJOINTS;i++){
				jnt_stiffness[i]=0.0;
				jnt_damping[i]=0.7;
			}
			setJointImpedance(jnt_stiffness, jnt_damping);

			static lwr_fri::FriJointTorques jnt_trqs_bias = current_jnt_trq;

			//calculating exerted torque and force
			for (int i = 0; i < NUMJOINTS; i++) {
				current_jnt_trq.msrJntTrq[i] -= jnt_trqs_bias.msrJntTrq[i];
				if(imp_update_counter%100==0)
				log(Info) << current_jnt_trq.msrJntTrq[i] << endlog();
			}

			//Torque
			if(imp_update_counter%100==0)
			log(Info) << "Setting efforts"
					<< current_jnt_trq.msrJntTrq[0] * TORQUE_FACTOR << endlog();
			motion_control_msgs::JointEfforts add_jnt_trq;

			for (int i = 0; i < NUMJOINTS; i++) {
				add_jnt_trq.efforts.push_back(current_jnt_trq.msrJntTrq[i] * TORQUE_FACTOR);
				add_jnt_trq.names.push_back("Joint");
			}
			if(imp_update_counter%100==0)
			log(Info) << "effort: " << add_jnt_trq.efforts.size() << endlog();

			port_cmd_addJntTrq.write(add_jnt_trq);

		}
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
	isStarted = true;
	port_isstarted_assistance.write(std_msgs::Empty { });
	log(Info) << "Proactive Assistance started" << endlog();
}
void ProactiveAssistance::stopAssistance(RTT::base::PortInterface*) {
	log(Info) << "Proactive Assistance about to stop!" << endlog();
	isStarted = false;
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

