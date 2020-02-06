#ifndef NEOKINEMATICSOMNIDRIVE_INCLUDEDEF_H
#define NEOKINEMATICSOMNIDRIVE_INCLUDEDEF_H
#include <ros/ros.h>
#include <neo_kinematics_omnidrive/ElmoMotorCtrl.h>
#include <neo_kinematics_omnidrive/DriveModule.h>
#include <neo_kinematics_omnidrive/NeoOmniDriveErrors.h>
#include <chrono>
//ros services
#include "neo_msgs/EmergencyStopState.h"
#include <neo_kinematics_omnidrive/Homing.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include "tf2/transform_datatypes.h"
#include <neo_kinematics_omnidrive/NeoKinematics.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include<NeoMath.h>
#include <sensor_msgs/JointState.h>
#include <neo_kinematics_omnidrive/SocketCan.h>
#include <neo_kinematics_omnidrive/CanMesg.h>
#include "std_msgs/Float64.h"




/*
 *params for all motor and gear combination
 */
struct Motorparams
{
	int     iTxPDO1;                // TPDO1 Transmit canopen i'd
	int     iTxPDO2;                // TPDO2 Transmit canopen i'd
	int     iRxPDO2;                // RPDO2 Receive canopen i'd   
	int     iTxSDO;                 // TSDO  Transmit canopen i'd  
	int     iRxSDO;                 // RSDO  Receive canopen  i'd
	int 	iEncIncrPerRevMot;      // encoder increments per revolution motor shaft 
	double	dVelMeasFrqHz;          // velocity measured in frequecy (hZ)s
	double	dGearRatio;             // Gear ratio   
	double	dBeltRatio;             // Belt ratio 
	int		iSign;                  // direction of motion
	double	dVelMaxEncIncrS;        // max velocity 
	double	dAccIncrS2;             // max acceleration
	double	dDecIncrS2;             // max deceleration 
	int	    iEncOffsetIncr;         // position in increments of steerwheel only when homing position is reached
	bool	bIsSteer;               // needed for distinguishing motor while initializing
	double  dCurrentToTorque;       // factor to convert motor active current [A] into torque [Nm] 
	double  dCurrMax;               // max current allowed   
	int 	iHomingDigIn;           // specifies the digital input for homing signal
	int     iHomingTimeout;         // Timeout value for homing
	int     iModulo;                // Modulo value 

};


/*
 *states of Drive
 */

enum StatesOfDrive
{
 ST_DRIVE_NOT_INIT,
 ST_DRIVE_INIT,
 ST_DRIVE_ERROR,
 ST_NOT_HOMED,
 ST_HOMING_FAILED,
 ST_SERVICE_CALLED,
 ST_STOPMOTION,
 ST_START_HOMING,
 ST_CONFIGURE_HOMING,
 ST_ARM_HOMING,
 ST_WAIT_FOR_HOMING,
 ST_ERROR_CORRECTION,
 ST_RECTIFYING,
 ST_RUNNING,
 ST_EMERGENCY
}; 

enum HomingStatus
{
  ARM=15,
  DISARM=16
};
//declaring objects for the motor params struct
Motorparams m_MotorSteer1;
Motorparams m_MotorDrive1;
Motorparams m_MotorSteer2;
Motorparams m_MotorDrive2;
Motorparams m_MotorSteer3;
Motorparams m_MotorDrive3;
Motorparams m_MotorSteer4;
Motorparams m_MotorDrive4;
//declaring object for Drive Module class
DriveModule DM1;
DriveModule DM2;
DriveModule DM3;
DriveModule DM4;


//variable for homing states
int m_iDriveState;
int m_iStoreState;

//declaring boolean variable flag to store the stop motion only once
bool bService_called;

//sleep time before arm homing
int iSleepTime;
int flag =0;
std::vector<bool> vBflag;
bool bEMstate = 0;

// Number of joints
int iNumOfJoints; 

//Variables for kinematic parameters
double dTransMaxVelocity, dRadMaxVelocity;

// Variable to check control initalisation
bool bIsIntialised = false;

// Geometrical Parameters
int iWheelDistMM;
int iWheelRadiusMM;
int iSteerAxisDistToDriveWheelMM;
double dCmdRateSec;
double dSpring;
double dDamp;
double dVirtualMass;
double dDPhiMax;
double dDDPhiMax;
double dMaxDriveRadS;
double dMaxSteerRadS;

// For calculation of ctrl_step
double dTimeout;
double dSample_time = 0.020;
int iWatchdog = 0;


// Position of the wheels steering axis in cartesian and polar co-ordinates relative to the robot co-ordinate system
std::vector<double> vdSteerPosWheelXMM;
std::vector<double> vdSteerPosWheelYMM;
std::vector<double> vdSteerWheelDistMM;
std::vector<double> vdSteerWheelAngRad;

// Position of the wheels in cartesian and polar co-ordinates relative to the robot co-ordinate system
std::vector<double> vdPosWheelXMM;
std::vector<double> vdPosWheelYMM;
std::vector<double> vdWheelDistMM;
std::vector<double> vdWheelAngRad;

// Neutral wheel Position 
std::vector<double> vdWheelNeutralPos;

// Flag for not running the state machine after homing
int iFST_Running = 0;



class NeoKinematicsOmniDrive
{

	public:
	// Publisher initialisation

		ros::Publisher pubJointStates;
		ros::Publisher pubJointControllerStates;
		ros::Publisher pub_Joint_controller;
		ros::Publisher pub_Odometry;
		ros::Publisher pub_Torque;
  		ros::NodeHandle n;   //ros node handle

	// Subscriber initialisation

		ros::Subscriber sub_Commanded_Twist;
		ros::Subscriber sub_Joint_States;
		ros::Subscriber sub_Joint_command;
		ros::Subscriber sub_JointStateCmd;
		ros::Time TOdomStamp;	

	// Object initialisation for the class NeoKinematics 

		NeoKinematics NC1;
		ros::Time last_time;
	  	int iTimeElapse;                                 //variable to store elapsed time
	 	std::chrono::steady_clock::time_point aStart,aEnd;
	// Variables for odom update
		double dPos_X =0, dPos_Y=0, dPos_Rad=0; 

		double dVel_X_last, dVel_Y_last, dVel_Rad_last;

		bool bBroadcast_tf;

	 	tf2_ros::TransformBroadcaster tfBroadcast_odometry;	

		ros::Time joint_state_odom_stamp;

		ros::Timer timer_ctrl_step_;

	// Constructor declaration 
		NeoKinematicsOmniDrive()
		{
		// Variable declaration


		// Publishers
		timer_ctrl_step_ = n.createTimer(ros::Duration(0.02), &NeoKinematicsOmniDrive::timerCallbackCtrlStep, this);


		// pubJointStates = n.advertise<sensor_msgs::JointState>("/joint_states", 100);

		pubJointControllerStates =  n.advertise<control_msgs::JointTrajectoryControllerState>("/controller_state", 1);


	// // 	// Navigation - Odometry 
		pub_Odometry = n.advertise<nav_msgs::Odometry>("/odometry", 1);

	// // Subscribers
		sub_JointStateCmd = n.subscribe("/joint_command", 1, &NeoKinematicsOmniDrive::topicCBJointCommand, this, ros::TransportHints().tcpNoDelay(true));

		// // Recieves the joint states. Used for inverse kinematics calculation.
		sub_Joint_States = n.subscribe("/controller_state", 1, &NeoKinematicsOmniDrive::topicCBJointStates, this, ros::TransportHints().tcpNoDelay(true)); 


		// 		// Joint trajectory controller allows us to control a joint by it's position, velocity or effort. 
		pub_Joint_controller =  n.advertise<control_msgs::JointTrajectoryControllerState> ("/joint_command", 1);

		// Subscribers for the user's command velocity, that the robot need to process.
		sub_Commanded_Twist = n.subscribe("/cmd_vel", 1, &NeoKinematicsOmniDrive::topicCBTwistCmd, this, ros::TransportHints().tcpNoDelay(true)); 

		pub_Torque = n.advertise<std_msgs::Float64>("measured_torque", 1);

		}

		~NeoKinematicsOmniDrive(){}

		// Setting up the callbacks

		void topicCBJointStates(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

		void topicCBTwistCmd(const geometry_msgs::Twist::ConstPtr& msg);

		void timerCallbackCtrlStep(const ros::TimerEvent& e); 

		void topicCBJointCommand(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

		// Setting up other functionalities

		bool JointStatesMeasure();

		void CalcCtrlStep();

		void Update_Odom();

		void JointTorqueMeasure();

		// void StartCommunication();


		

};
#endif