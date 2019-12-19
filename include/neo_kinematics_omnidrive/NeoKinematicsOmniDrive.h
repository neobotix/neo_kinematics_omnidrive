#ifndef NEOKINEMATICSOMNIDRIVE_INCLUDEDEF_H
#define NEOKINEMATICSOMNIDRIVE_INCLUDEDEF_H
#include <ros/ros.h>
#include <neo_kinematics_omnidrive/ElmoMotorCtrl.h>
#include <neo_kinematics_omnidrive/DriveModule.h>
#include <neo_kinematics_omnidrive/NeoOmniDriveErrors.h>
#include <chrono>
//ros services
#include <neo_kinematics_omnidrive/Homing.h>
#include<vector>
#include "neo_msgs/EmergencyStopState.h"

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


//variable for homing statess
int m_iDriveState;
int m_iStoreState;

//declaring boolean variable flag to store the stop motion only once
bool bService_called;

//sleep time before arm homing
int iSleepTime;
int flag =0;
std::vector<bool> vBflag;
bool bEMstate = 0;

#endif
