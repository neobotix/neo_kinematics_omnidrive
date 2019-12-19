#ifndef ELMOMOTORCTRL_INCLUDEDEF_H
#define ELMOMOTORCTRL_INCLUDEDEF_H

#include <neo_kinematics_omnidrive/CanMesg.h>
#include <neo_kinematics_omnidrive/DriveParameters.h>
#include <neo_kinematics_omnidrive/SocketCan.h>


class ElmoMotorCtrl
{
public:
int flag = 0;

/**
 *default constructor
 */
ElmoMotorCtrl();


/**
 *default destructor
 */
~ElmoMotorCtrl();
 

/**
 * Id's of the CAN messages.
 */
struct ParamCanopen
{
  int iTxPDO1;
  int iTxPDO2;
  int iRxPDO2;
  int iTxSDO;
  int iRxSDO;
};

enum HomingStatus
{
  ARM=15,
  DISARM=16
};
/**
 * Sets the CAN Id's
 * @param iTxPDO1: 1st transmit process data object
 * @param iTxPDO2: 2nd transmit process data object
 * @param iRxPDO2: second receive process data object
 * @param iTxSDO: transmit service data object
 * @param iRxSDO: receive service data object
 */
void setCanOpenParam( int iTxPDO1, int iTxPDO2, int iRxPDO2, int iTxSDO, int iRxSDO);


/**
 *Initializes the motor controller , by checking modulo condition and sets velocity motion control &finally configures drives using SDO download
 *call this function after constructor is invoked
 */
int initMotorCtrl();


/**
 *Interpreter comand sent in binary form used for setting  all numerical data to servo drives
 *@param iDatalen: length of data (int)
 *@param cmdchar1: first command character(char)
 *@param cmdchar2: second command character(char)
 *@param iIndex: Index of an array (int)
 *@param iData: data (int)
 */
void setInterpreter(int iDatalen,char cmdchar1,char cmdchar2,int iIndex,int iData );


/**
  *initialises the Homing
  */
int configureHoming();

/**
 *Performs the Homing procedure
 */ 
int homingDone();


/**
 *arm homing
 */
void armHoming();


/**
 *Turning the motor on
 */
int turnOnMotor();


/**
 *Turning the motor off
 */
bool turnOffMotor();


/**
 *sets the velocity in Rad/s
 *@param dGearvelrads: vel(double)
 */
void setVelInRadS(double dGearvelrads);



/*
 *function for stopping motion in rads
 */
void stopMotion();

/**
 *returns a bit-field reporthing the status of the system in concise format
 *@ param iStatus: bit which it got (integer)
 */
int evaluateStatusRegister(int iStatus);

bool TriggeredCondition();

/**
 *evaluates the message received
 *only messages with matching Id's are evaluated
 *mesage evaluated in object mode
 */
std::vector <int> evaluatingMessageReceived();


/**
 *it will reply true if elmo disarmed after the homing event if not viceversa
 */
bool getStatusOfHoming();


/**
 *checks if recevied bit is equal to failure message of corresponding bit
 */
bool isBitSet(int iValue, int iBit);


/**
 *CANopen: writes a service data object from master to device in expedited transfer mode(only 1 message at a time)
 */
void sendingSDODownload(int iIndex, int iSubindex, int iData);


/**
 *sending CAN message frame format
 *@param iId: can frame id(int)
 *@param iLen: can frame lenght (int)
 *@param cByte: first byte of message byte (char)
 */
void sendCanMessage(int iId, int iLen, unsigned char cByte);



/**
 *getting gear positon and velocity
 *@param dPositonGearMeasInRad : gear positon measurement in radians
 *@param dVelGearMeasInRadS : gear velocity measurement in rad/s
 */
void getGearPosAndVel(double *pdPositonGearMeasInRad, double *pdVelGearMeasInRadS);


/**
 *get gear vel
 *@param dVelGearMeasInRadS : gear velocity measurement in rad/s
 */
void getGearVel(double *pdVelGearMeasInRadS);

void getGearTor(double *pdTorGear);

void setGearTor(double dTorqueNm);

bool get_limitswitch_state();

/**
 *sets the drive parameters by passing objects 
 */
void settingDriveParams(DriveParameters DrParam){m_DriveParameter=DrParam;}

bool m_bLimSwRight;



private:

/*
 *objects for classes
 */
SocketCan* m_sCanCtrl;
DriveParameters m_DriveParameter;
/*
 *params
 */
ParamCanopen m_ParamCanopen;

/*
 *declaration of variables
 */
double m_dPositonGearMeasInRad;  // position  gear measurement in radians
double m_dVelGearMeasInRadS;     // velocity gear measurement in rad/s
double m_dTorGear;
bool m_bHomingStatus;           // tells whether elmo disarmed after homing performed or not
int m_iStatus;                  // status of message      
double m_dMotorCurr;
double dTorqueConstant = 0.116;  //Torque constant 0.116 Nm/A
unsigned int m_iDigIn;
};
#endif
