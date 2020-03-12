#ifndef DRIVEMODULE_INCLUDEDEF_H
#define DRIVEMODULE_INCLUDEDEF_H
#include <chrono>

#include <neo_kinematics_omnidrive/CanMesg.h>
#include <neo_kinematics_omnidrive/ElmoMotorCtrl.h>
#include <neo_kinematics_omnidrive/DriveParameters.h>
//Services


class DriveModule
{
public:

/*
 *default constructor
 */
DriveModule();

/*
 *default destructor
 */
~DriveModule();

/*
 *function which sets all Id's & params for steer and drive
 */
int init(
	      int TxPDO1_WSteer, int TxPDO2_WSteer, int RxPDO2_WSteer, int TxSDO_WSteer, int RxSDO_WSteer, int 	Steer_iEncIncrPerRevMot, double	Steer_dVelMeasFrqHz,
	      double Steer_dGearRatio, double Steer_dBeltRatio, int	 Steer_iSign, double Steer_dVelMaxEncIncrS, double	Steer_dAccIncrS2, double Steer_dDecIncrS2,
	      int Steer_iEncOffsetIncr, bool Steer_bIsSteer, double  Steer_dCurrentToTorque, double  Steer_dCurrMax, int  Steer_iHomingDigIn,int Steer_iHomingTimeout,int Steer_iModulo, 
	      int TxPDO1_WDrive, int TxPDO2_WDrive, int RxPDO2_WDrive, int TxSDO_WDrive, int RxSDO_WDrive, int 	Drive_iEncIncrPerRevMot, double	Drive_dVelMeasFrqHz,
	      double Drive_dGearRatio, double Drive_dBeltRatio, int	 Drive_iSign, double Drive_dVelMaxEncIncrS, double	Drive_dAccIncrS2, double Drive_dDecIncrS2,
	      int Drive_iEncOffsetIncr, bool Drive_bIsSteer, double  Drive_dCurrentToTorque, double  Drive_dCurrMax, int  Drive_iHomingDigIn,int Drive_iHomingTimeout,int Drive_iModulo 
	      );



/*
 *function which initializes the homing
 */
int configureHoming();

/*
 *function which performs the homing procedure by calling the corresponding function in Elmomotor class
 */
int homingDone();

bool TriggeredCondition();

/**
 *arm homing
 */
void armHoming();


/*
 *function for stopping motion in rads
 */
void stopMotion();

/*
 *function for setting velocity in rads
 */
void setVelInRadS(int i, double dGearvelrads);

/*
 *function for starting network management 
 */
int sendNetStartCanOpen();


/*
 *get gear position and velocity
 */
void getGearPosAndVel(int i,double *pdPosGearRad, double *pdVelGearRadS);

/*
 *get gearvelocity
 */
void getGearVel(int i, double *pdVelGearRadS);

void getGearTor(int i, double *pdTorGear);

void setGearTor( double dTorqueNm);

void OpenSocket();

void startCommunication();

// bool SrvMotorSwitchCB(neo_kinematics_omnidrive::MotorSwitch::Request  &req, neo_kinematics_omnidrive::MotorSwitch::Response &res);

/*
 *it receives all the messages and evaluates it
 */
std::vector <int> recMessages();

private:

//canopen ids for drive1
int TxPDO1_WDrive,TxPDO2_WDrive,RxPDO2_WDrive,TxSDO_WDrive,RxSDO_WDrive;
//canopen ids for steer1
int TxPDO1_WSteer,TxPDO2_WSteer,RxPDO2_WSteer,TxSDO_WSteer,RxSDO_WSteer;

//params for all steer motor and gear combination

int 	Steer_iEncIncrPerRevMot;            // encoder increments per revolution motor shaft 
double	Steer_dVelMeasFrqHz;                // velocity measured in frequecy (hZ)
double	Steer_dGearRatio;                   // Gear ratio   
double	Steer_dBeltRatio;                   // Belt ratio 
int		Steer_iSign;                        // direction of motion
double	Steer_dVelMaxEncIncrS;              // max velocity 
double	Steer_dAccIncrS2;                   // max acceleration
double	Steer_dDecIncrS2;                   // max deceleration
int	    Steer_iEncOffsetIncr;               // position in increments of steerwheel only when homing position is reached
bool	Steer_bIsSteer;                     // needed for distinguishing motor while initializing
double  Steer_dCurrentToTorque;             // factor to convert motor active current [A] into torque [Nm] 
double  Steer_dCurrMax;                     // max current allowed   
int 	Steer_iHomingDigIn;                 // specifies the digital input for homing signal
int     Steer_iHomingTimeout;               // steer Homing time out
int     Steer_iModulo;                       // steer modulo


//params for all steer motor and gear combination

int 	Drive_iEncIncrPerRevMot;            // encoder increments per revolution motor shaft 
double	Drive_dVelMeasFrqHz;                // velocity measured in frequecy (hZ)
double	Drive_dGearRatio;                   // Gear ratio   
double	Drive_dBeltRatio;                   // Belt ratio 
int		Drive_iSign;                        // direction of motion
double	Drive_dVelMaxEncIncrS;              // max velocity 
double	Drive_dAccIncrS2;                   // max acceleration
double	Drive_dDecIncrS2;                   // max deceleration
int	    Drive_iEncOffsetIncr;               // position in increments of Drivewheel only when homing position is reached
bool	Drive_bIsSteer;                     // needed for distinguishing motor while initializing
double  Drive_dCurrentToTorque;             // factor to convert motor active current [A] into torque [Nm] 
double  Drive_dCurrMax;                     // max current allowed   
int 	Drive_iHomingDigIn;                 // specifies the digital input for homing signal
int     Drive_iHomingTimeout;               // steer Homing time out
int     Drive_iModulo;                       // steer modulo





//declaring steer and drive as objects for ElmoMotorCtrl class
ElmoMotorCtrl Steer;
ElmoMotorCtrl Drive;

//declaring steer and drive as objects for DriveParameters class
DriveParameters m_steerParam;
DriveParameters m_driveParam;
int iTimeElapsed,iTimeSleep, iTimeSleep1, iTimeSleep2;
std::chrono::steady_clock::time_point t1, t2;




};
#endif
