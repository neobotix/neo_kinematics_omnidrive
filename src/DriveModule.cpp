
#include <neo_kinematics_omnidrive/DriveModule.h>
#include <unistd.h>
#include <neo_kinematics_omnidrive/ElmoMotorCtrl.h>


DriveModule::DriveModule()
{
  // Steer=new ElmoMotorCtrl();    //heap declaration of object for ElmoMotorCtrl class
  // Drive=new ElmoMotorCtrl();    //heap declaration of object for ElmoMotorCtrl class
 
}

DriveModule::~DriveModule()
{
 
}


int DriveModule::sendNetStartCanOpen()
{ 
  Drive.sendCanMessage(0, 2, 1);     //start network command brings the drive to operational stage               
  return 0;
}

int DriveModule::init(
                       int TxPDO1_WSteer, int TxPDO2_WSteer, int RxPDO2_WSteer, int TxSDO_WSteer, int RxSDO_WSteer, int  Steer_iEncIncrPerRevMot, double Steer_dVelMeasFrqHz,
                       double Steer_dGearRatio, double Steer_dBeltRatio, int  Steer_iSign, double Steer_dVelMaxEncIncrS, double  Steer_dAccIncrS2, double Steer_dDecIncrS2,
                       int Steer_iEncOffsetIncr, bool Steer_bIsSteer, double  Steer_dCurrentToTorque, double  Steer_dCurrMax, int  Steer_iHomingDigIn,int Steer_iHomingTimeout,int Steer_iModulo, 
                       int TxPDO1_WDrive, int TxPDO2_WDrive, int RxPDO2_WDrive, int TxSDO_WDrive, int RxSDO_WDrive, int  Drive_iEncIncrPerRevMot, double Drive_dVelMeasFrqHz,
                       double Drive_dGearRatio, double Drive_dBeltRatio, int  Drive_iSign, double Drive_dVelMaxEncIncrS, double  Drive_dAccIncrS2, double Drive_dDecIncrS2,
                       int Drive_iEncOffsetIncr, bool Drive_bIsSteer, double  Drive_dCurrentToTorque, double  Drive_dCurrMax, int  Drive_iHomingDigIn,int Drive_iHomingTimeout,int Drive_iModulo 
                     )
{
  int iDriveRet = 0;                   //variable to store return value of Drive initializing motor
  int iSteerRet = 0;                   //variable to store return value of Steer initializing motor
  int iDriveMotorOn=0;                 //variable to store return value of Drive motor turn on 
  int iSteerMotorOn=0;                 //variable to store return value of Steer motor turn on 
  
 

//setting params of Drivemotor in driveparams class
  m_driveParam.settingParams
  (
      0,
      Drive_iEncIncrPerRevMot,
      Drive_dVelMeasFrqHz,
      Drive_dBeltRatio,
      Drive_dGearRatio,
      Drive_iSign,
      Drive_dVelMaxEncIncrS,
      Drive_dAccIncrS2,
      Drive_dDecIncrS2,
      Drive_iEncOffsetIncr,
      Drive_bIsSteer,
      Drive_dCurrentToTorque,
      Drive_dCurrMax,
      Drive_iHomingDigIn,
      Drive_iHomingTimeout,
      Drive_iModulo

  );
  


  //setting params of steermotor in driveparams class

  m_steerParam.settingParams
  (   1,
      Steer_iEncIncrPerRevMot,
      Steer_dVelMeasFrqHz,
      Steer_dGearRatio,
      Steer_dBeltRatio,
      Steer_iSign,
      Steer_dVelMaxEncIncrS,
      Steer_dAccIncrS2,
      Steer_dDecIncrS2,
      Steer_iEncOffsetIncr,
      Steer_bIsSteer,
      Steer_dCurrentToTorque,
      Steer_dCurrMax,
      Steer_iHomingDigIn,
      Steer_iHomingTimeout,
      Steer_iModulo

  );


  //setting the Drive motor canoopen params
  Drive.setCanOpenParam(TxPDO1_WDrive,TxPDO2_WDrive,RxPDO2_WDrive,TxSDO_WDrive,RxSDO_WDrive);

  //setting the Steer motor params
  Drive.settingDriveParams(m_driveParam);

  //setting the steer motor canopoen params
  Steer.setCanOpenParam(TxPDO1_WSteer,TxPDO2_WSteer,RxPDO2_WSteer,TxSDO_WSteer,RxSDO_WSteer);

  //setting the steer motor params 
  Steer.settingDriveParams(m_steerParam);
  
  
  //checking if there is any error with drive motor initialization so that program can terminate
  if((iDriveRet = Drive.initMotorCtrl())>0)
  {
     return iDriveRet;
  }

  //checking if there is any error with Steer motor initialization so that program can terminate
  if((iSteerRet = Steer.initMotorCtrl())>0)
  {
    return iSteerRet;
  }
  

  // turn on Drive motor only when drive motor is initialized successufully
  if(iDriveRet==0)
  {

    if((iDriveMotorOn=Drive.turnOnMotor())>0)
    {
      return iDriveMotorOn;
    }  
  }

  // turn on Steer motor only when Steer motor is initialized successufully
  if(iSteerRet==0)
  {
    if((iSteerMotorOn=Steer.turnOnMotor())>0)
    {

      return iSteerMotorOn;
    }  
  }
  // Drive.turnOffMotor();
  // Steer.turnOffMotor();
  return 0;
}

// bool DriveModule::SrvMotorSwitchCB(neo_kinematics_omnidrive::MotorSwitch::Request  &req, neo_kinematics_omnidrive::MotorSwitch::Response &res)
// {
//   Drive.turnOffMotor();
//   Steer.turnOffMotor();
//   res.Switch.data = false;
//   return true;
// }

int DriveModule::configureHoming()
{
  int iRet;      //return variable which stores configureHoming funciton return value

  //execute homing for the steer motor 
  iRet=Steer.configureHoming();

  return iRet;

}

void DriveModule::armHoming()
{

  //execute homing for the steer motor 
  Steer.armHoming();

}

int DriveModule::homingDone()
{
  int iRet;      //return variable which stores executeHoming funciton return value

  //execute homing for the steer motor 
  iRet=Steer.homingDone();

  return iRet;

}

void DriveModule::setVelInRadS(int i, double dGearvelrads)
{

  //setting veloicty of motor in rads 
  if(i == 0)

  {Drive.setVelInRadS(dGearvelrads);}

  if(i == 1)

  {Steer.setVelInRadS(dGearvelrads);}
 
}


void DriveModule::stopMotion()
{
  //call the sotp motion funciotn from ElmoMotorCtrl class
  Drive.stopMotion();   
  Steer.stopMotion();
 
}


void DriveModule::getGearPosAndVel(int i,double *pdPosGearRad, double *pdVelGearRadS)
{
  *pdPosGearRad=0;
  *pdVelGearRadS=0;
  
  // for i=0, it gets pos and vel for Drive motor
  if(i==0) 
  {
    Drive.getGearPosAndVel(pdPosGearRad, pdVelGearRadS);
  }

  // for i=1, it gets pos and vel for Steer motor
  if (i==1)
  {
    Steer.getGearPosAndVel(pdPosGearRad, pdVelGearRadS);
  }
}


void DriveModule::getGearVel(int i,double *pdVelGearRadS)
{
  *pdVelGearRadS=0;
  
  // for i=0, it gets pos and vel for Drive motor
  if(i==0) 
  {
    Drive.getGearVel(pdVelGearRadS);
  }

  // for i=1, it gets pos and vel for Steer motor
  if (i==1)
  {
    Steer.getGearVel(pdVelGearRadS);
  }
}

void DriveModule::getGearTor(int i,double *pdTorGear)
{
  *pdTorGear=0;
  
  // for i=0, it gets torque for Drive motor
  if(i==0) 
  {
    Drive.getGearTor(pdTorGear);
  }

  // for i=1, it gets torque for Steer motor
  if (i==1)
  {
    Steer.getGearTor(pdTorGear);
  }
}

bool DriveModule::TriggeredCondition()
{
  Steer.TriggeredCondition();
}

void DriveModule::setGearTor(double dTorqueNm)
{
  std::cout<<"Here";
  Drive.setGearTor(dTorqueNm);
  // Steer.setGearTor(dGearvelrads);
}


std::vector <int> DriveModule::recMessages()
{
  std::vector <int> viReturn;
  //evlauating the recevied messges
  t1 = std::chrono::steady_clock::now();

  viReturn= Drive.evaluatingMessageReceived();

  viReturn=Steer.evaluatingMessageReceived();

  auto t2 = std::chrono::steady_clock::now();
  iTimeSleep1 = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  // std::cout<<iTimeSleep1<<std::endl;
  return viReturn;

}

void  DriveModule::OpenSocket()
{
  Drive.OpenSocket();

    // Steer.OpenSocket();

}