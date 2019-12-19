#include <ros/ros.h>
#include <neo_kinematics_omnidrive/NeoKinematicsOmniDrive.h>
#include "std_msgs/Float64.h"
#include <fstream>

/*
 *loading all the neccesary parameters from yaml file
 */
 void velocityCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    // DM1.setVelInRadS(msg->data);
    // DM2.setVelInRadS(msg->data);
    // DM3.setVelInRadS(msg->data);
    // DM4.setVelInRadS(msg->data);

  }
 void TorqueCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    DM1.setGearTor(msg->data);
  }

void EmgCB(const neo_msgs::EmergencyStopState msg)
{
  if((bool)msg.emergency_button_stop || (bool)msg.scanner_stop == 1)
  {
    bEMstate = 1;
  }
  else
  {
    bEMstate = 0;
  }
}
int loadingParams(const ros::NodeHandle &n)
{


  if (n.hasParam("Time/Sleeptime"))
  {
    n.getParam("Time/Sleeptime", iSleepTime);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-Modulo parameter from parameter server");
    return 1;
  }

  //-----------------------------steer1---------------------------------------------------------
  // loading steer1 params
  if (n.hasParam("CanOpenIDs/TxPDO1_W1Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W1Steer", m_MotorSteer1.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W1Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W1Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W1Steer", m_MotorSteer1.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W1Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W1Steer"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W1Steer", m_MotorSteer1.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W1Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W1Steer"))
  {
    n.getParam("CanOpenIDs/TxSDO_W1Steer", m_MotorSteer1.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W1Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W1Steer"))
  {
    n.getParam("CanOpenIDs/RxSDO_W1Steer", m_MotorSteer1.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W1Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/EncIncrPerRevMot"))
  {
    n.getParam("Steer1/EncIncrPerRevMot", m_MotorSteer1.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/VelMeasFrqHz"))
  {
    n.getParam("Steer1/VelMeasFrqHz", m_MotorSteer1.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/BeltRatio"))
  {
    n.getParam("Steer1/BeltRatio", m_MotorSteer1.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/GearRatio"))
  {
    n.getParam("Steer1/GearRatio", m_MotorSteer1.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/Sign"))
  {
    n.getParam("Steer1/Sign", m_MotorSteer1.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/VelMaxEncIncrS"))
  {
    n.getParam("Steer1/VelMaxEncIncrS", m_MotorSteer1.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/AccIncrS"))
  {
    n.getParam("Steer1/AccIncrS", m_MotorSteer1.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/DecIncrS"))
  {
    n.getParam("Steer1/DecIncrS", m_MotorSteer1.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/EncOffsetIncr"))
  {
    n.getParam("Steer1/EncOffsetIncr", m_MotorSteer1.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/IsSteering"))
  {
    n.getParam("Steer1/IsSteering", m_MotorSteer1.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/CurrentToTorque"))
  {
    n.getParam("Steer1/CurrentToTorque", m_MotorSteer1.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/CurrMax"))
  {
    n.getParam("Steer1/CurrMax", m_MotorSteer1.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/CurrMax"))
  {
    n.getParam("Steer1/CurrMax", m_MotorSteer1.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/HomingDigIn"))
  {
    n.getParam("Steer1/HomingDigIn", m_MotorSteer1.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/HomingTimeout"))
  {
    n.getParam("Steer1/HomingTimeout", m_MotorSteer1.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer1/Modulo"))
  {
    n.getParam("Steer1/Modulo", m_MotorSteer1.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer1-Modulo parameter from parameter server");
    return 1;
  }






//---------------------drive1------------------------------------------------------------------------
  //loading drive params

  if (n.hasParam("CanOpenIDs/TxPDO1_W1Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W1Drive", m_MotorDrive1.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W1Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W1Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W1Drive", m_MotorDrive1.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W1Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W1Drive"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W1Drive", m_MotorDrive1.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W1Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W1Drive"))
  {
    n.getParam("CanOpenIDs/TxSDO_W1Drive", m_MotorDrive1.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W1Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W1Drive"))
  {
    n.getParam("CanOpenIDs/RxSDO_W1Drive", m_MotorDrive1.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W1Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/EncIncrPerRevMot"))
  {
    n.getParam("Drive1/EncIncrPerRevMot", m_MotorDrive1.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/VelMeasFrqHz"))
  {
    n.getParam("Drive1/VelMeasFrqHz", m_MotorDrive1.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/BeltRatio"))
  {
    n.getParam("Drive1/BeltRatio", m_MotorDrive1.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/GearRatio"))
  {
    n.getParam("Drive1/GearRatio", m_MotorDrive1.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/Sign"))
  {
    n.getParam("Drive1/Sign", m_MotorDrive1.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/VelMaxEncIncrS"))
  {
    n.getParam("Drive1/VelMaxEncIncrS", m_MotorDrive1.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/AccIncrS"))
  {
    n.getParam("Drive1/AccIncrS", m_MotorDrive1.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/DecIncrS"))
  {
    n.getParam("Drive1/DecIncrS", m_MotorDrive1.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/EncOffsetIncr"))
  {
    n.getParam("Drive1/EncOffsetIncr", m_MotorDrive1.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/IsSteering"))
  {
    n.getParam("Drive1/IsSteering", m_MotorDrive1.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/CurrentToTorque"))
  {
    n.getParam("Drive1/CurrentToTorque", m_MotorDrive1.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/CurrMax"))
  {
    n.getParam("Drive1/CurrMax", m_MotorDrive1.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/HomingDigIn"))
  {
    n.getParam("Drive1/HomingDigIn", m_MotorDrive1.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/HomingTimeout"))
  {
    n.getParam("Drive1/HomingTimeout", m_MotorDrive1.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive1/Modulo"))
  {
    n.getParam("Drive1/Modulo", m_MotorDrive1.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive1-Modulo parameter from parameter server");
    return 1;
  }



  //-----------------------------steer2---------------------------------------------------------
  // loading steer1 params
  if (n.hasParam("CanOpenIDs/TxPDO1_W2Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W2Steer", m_MotorSteer2.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W2Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W2Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W2Steer", m_MotorSteer2.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W2Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W2Steer"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W2Steer", m_MotorSteer2.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W2Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W2Steer"))
  {
    n.getParam("CanOpenIDs/TxSDO_W2Steer", m_MotorSteer2.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W2Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W2Steer"))
  {
    n.getParam("CanOpenIDs/RxSDO_W2Steer", m_MotorSteer2.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W2Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/EncIncrPerRevMot"))
  {
    n.getParam("Steer2/EncIncrPerRevMot", m_MotorSteer2.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/VelMeasFrqHz"))
  {
    n.getParam("Steer2/VelMeasFrqHz", m_MotorSteer2.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/BeltRatio"))
  {
    n.getParam("Steer2/BeltRatio", m_MotorSteer2.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/GearRatio"))
  {
    n.getParam("Steer2/GearRatio", m_MotorSteer2.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/Sign"))
  {
    n.getParam("Steer2/Sign", m_MotorSteer2.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/VelMaxEncIncrS"))
  {
    n.getParam("Steer2/VelMaxEncIncrS", m_MotorSteer2.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/AccIncrS"))
  {
    n.getParam("Steer2/AccIncrS", m_MotorSteer2.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/DecIncrS"))
  {
    n.getParam("Steer2/DecIncrS", m_MotorSteer2.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/EncOffsetIncr"))
  {
    n.getParam("Steer2/EncOffsetIncr", m_MotorSteer2.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/IsSteering"))
  {
    n.getParam("Steer2/IsSteering", m_MotorSteer2.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/CurrentToTorque"))
  {
    n.getParam("Steer2/CurrentToTorque", m_MotorSteer2.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/CurrMax"))
  {
    n.getParam("Steer2/CurrMax", m_MotorSteer2.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/CurrMax"))
  {
    n.getParam("Steer2/CurrMax", m_MotorSteer2.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/HomingDigIn"))
  {
    n.getParam("Steer2/HomingDigIn", m_MotorSteer2.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/HomingTimeout"))
  {
    n.getParam("Steer2/HomingTimeout", m_MotorSteer2.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer2/Modulo"))
  {
    n.getParam("Steer2/Modulo", m_MotorSteer2.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer2-Modulo parameter from parameter server");
    return 1;
  }

//---------------------drive2------------------------------------------------------------------------
  //loading drive params

  if (n.hasParam("CanOpenIDs/TxPDO1_W2Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W2Drive", m_MotorDrive2.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W2Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W2Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W2Drive", m_MotorDrive2.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W2Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W2Drive"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W2Drive", m_MotorDrive2.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W2Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W2Drive"))
  {
    n.getParam("CanOpenIDs/TxSDO_W2Drive", m_MotorDrive2.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W2Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W2Drive"))
  {
    n.getParam("CanOpenIDs/RxSDO_W2Drive", m_MotorDrive2.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W2Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/EncIncrPerRevMot"))
  {
    n.getParam("Drive2/EncIncrPerRevMot", m_MotorDrive2.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/VelMeasFrqHz"))
  {
    n.getParam("Drive2/VelMeasFrqHz", m_MotorDrive2.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/BeltRatio"))
  {
    n.getParam("Drive2/BeltRatio", m_MotorDrive2.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/GearRatio"))
  {
    n.getParam("Drive2/GearRatio", m_MotorDrive2.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/Sign"))
  {
    n.getParam("Drive2/Sign", m_MotorDrive2.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/VelMaxEncIncrS"))
  {
    n.getParam("Drive2/VelMaxEncIncrS", m_MotorDrive2.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/AccIncrS"))
  {
    n.getParam("Drive2/AccIncrS", m_MotorDrive2.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/DecIncrS"))
  {
    n.getParam("Drive2/DecIncrS", m_MotorDrive2.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/EncOffsetIncr"))
  {
    n.getParam("Drive2/EncOffsetIncr", m_MotorDrive2.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/IsSteering"))
  {
    n.getParam("Drive2/IsSteering", m_MotorDrive2.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/CurrentToTorque"))
  {
    n.getParam("Drive2/CurrentToTorque", m_MotorDrive2.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/CurrMax"))
  {
    n.getParam("Drive2/CurrMax", m_MotorDrive2.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/HomingDigIn"))
  {
    n.getParam("Drive2/HomingDigIn", m_MotorDrive2.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/HomingTimeout"))
  {
    n.getParam("Drive2/HomingTimeout", m_MotorDrive2.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive2/Modulo"))
  {
    n.getParam("Drive2/Modulo", m_MotorDrive2.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive2-Modulo parameter from parameter server");
    return 1;
  }



  //-----------------------------steer3---------------------------------------------------------
  // loading steer1 params
  if (n.hasParam("CanOpenIDs/TxPDO1_W3Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W3Steer", m_MotorSteer3.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W3Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W3Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W3Steer", m_MotorSteer3.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W3Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W3Steer"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W3Steer", m_MotorSteer3.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W3Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W3Steer"))
  {
    n.getParam("CanOpenIDs/TxSDO_W3Steer", m_MotorSteer3.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W3Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W3Steer"))
  {
    n.getParam("CanOpenIDs/RxSDO_W3Steer", m_MotorSteer3.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W3Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/EncIncrPerRevMot"))
  {
    n.getParam("Steer3/EncIncrPerRevMot", m_MotorSteer3.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/VelMeasFrqHz"))
  {
    n.getParam("Steer3/VelMeasFrqHz", m_MotorSteer3.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/BeltRatio"))
  {
    n.getParam("Steer3/BeltRatio", m_MotorSteer3.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/GearRatio"))
  {
    n.getParam("Steer3/GearRatio", m_MotorSteer3.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/Sign"))
  {
    n.getParam("Steer3/Sign", m_MotorSteer3.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/VelMaxEncIncrS"))
  {
    n.getParam("Steer3/VelMaxEncIncrS", m_MotorSteer3.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/AccIncrS"))
  {
    n.getParam("Steer3/AccIncrS", m_MotorSteer3.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/DecIncrS"))
  {
    n.getParam("Steer3/DecIncrS", m_MotorSteer3.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/EncOffsetIncr"))
  {
    n.getParam("Steer3/EncOffsetIncr", m_MotorSteer3.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/IsSteering"))
  {
    n.getParam("Steer3/IsSteering", m_MotorSteer3.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/CurrentToTorque"))
  {
    n.getParam("Steer3/CurrentToTorque", m_MotorSteer3.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/CurrMax"))
  {
    n.getParam("Steer3/CurrMax", m_MotorSteer3.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/CurrMax"))
  {
    n.getParam("Steer3/CurrMax", m_MotorSteer3.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/HomingDigIn"))
  {
    n.getParam("Steer3/HomingDigIn", m_MotorSteer3.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/HomingTimeout"))
  {
    n.getParam("Steer3/HomingTimeout", m_MotorSteer3.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer3/Modulo"))
  {
    n.getParam("Steer3/Modulo", m_MotorSteer3.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer3-Modulo parameter from parameter server");
    return 1;
  }

//---------------------drive3------------------------------------------------------------------------
  //loading drive params

  if (n.hasParam("CanOpenIDs/TxPDO1_W3Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W3Drive", m_MotorDrive3.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W3Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W3Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W3Drive", m_MotorDrive3.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W3Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W3Drive"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W3Drive", m_MotorDrive3.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W3Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W3Drive"))
  {
    n.getParam("CanOpenIDs/TxSDO_W3Drive", m_MotorDrive3.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W3Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W3Drive"))
  {
    n.getParam("CanOpenIDs/RxSDO_W3Drive", m_MotorDrive3.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W3Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/EncIncrPerRevMot"))
  {
    n.getParam("Drive3/EncIncrPerRevMot", m_MotorDrive3.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/VelMeasFrqHz"))
  {
    n.getParam("Drive3/VelMeasFrqHz", m_MotorDrive3.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/BeltRatio"))
  {
    n.getParam("Drive3/BeltRatio", m_MotorDrive3.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/GearRatio"))
  {
    n.getParam("Drive3/GearRatio", m_MotorDrive3.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/Sign"))
  {
    n.getParam("Drive3/Sign", m_MotorDrive3.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/VelMaxEncIncrS"))
  {
    n.getParam("Drive3/VelMaxEncIncrS", m_MotorDrive3.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/AccIncrS"))
  {
    n.getParam("Drive3/AccIncrS", m_MotorDrive3.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/DecIncrS"))
  {
    n.getParam("Drive3/DecIncrS", m_MotorDrive3.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/EncOffsetIncr"))
  {
    n.getParam("Drive3/EncOffsetIncr", m_MotorDrive3.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/IsSteering"))
  {
    n.getParam("Drive3/IsSteering", m_MotorDrive3.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/CurrentToTorque"))
  {
    n.getParam("Drive3/CurrentToTorque", m_MotorDrive3.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/CurrMax"))
  {
    n.getParam("Drive3/CurrMax", m_MotorDrive3.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/HomingDigIn"))
  {
    n.getParam("Drive3/HomingDigIn", m_MotorDrive3.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/HomingTimeout"))
  {
    n.getParam("Drive3/HomingTimeout", m_MotorDrive3.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive3/Modulo"))
  {
    n.getParam("Drive3/Modulo", m_MotorDrive3.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive3-Modulo parameter from parameter server");
    return 1;
  }


//-----------------------------steer4---------------------------------------------------------
  // loading steer4 params
  if (n.hasParam("CanOpenIDs/TxPDO1_W4Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W4Steer", m_MotorSteer4.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W4Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W4Steer"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W4Steer", m_MotorSteer4.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W4Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W4Steer"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W4Steer", m_MotorSteer4.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W4Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W4Steer"))
  {
    n.getParam("CanOpenIDs/TxSDO_W4Steer", m_MotorSteer4.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W4Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W4Steer"))
  {
    n.getParam("CanOpenIDs/RxSDO_W4Steer", m_MotorSteer4.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W4Steer parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/EncIncrPerRevMot"))
  {
    n.getParam("Steer4/EncIncrPerRevMot", m_MotorSteer4.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/VelMeasFrqHz"))
  {
    n.getParam("Steer4/VelMeasFrqHz", m_MotorSteer4.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/BeltRatio"))
  {
    n.getParam("Steer4/BeltRatio", m_MotorSteer4.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/GearRatio"))
  {
    n.getParam("Steer4/GearRatio", m_MotorSteer4.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/Sign"))
  {
    n.getParam("Steer4/Sign", m_MotorSteer4.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/VelMaxEncIncrS"))
  {
    n.getParam("Steer4/VelMaxEncIncrS", m_MotorSteer4.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/AccIncrS"))
  {
    n.getParam("Steer4/AccIncrS", m_MotorSteer4.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/DecIncrS"))
  {
    n.getParam("Steer4/DecIncrS", m_MotorSteer4.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/EncOffsetIncr"))
  {
    n.getParam("Steer4/EncOffsetIncr", m_MotorSteer4.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/IsSteering"))
  {
    n.getParam("Steer4/IsSteering", m_MotorSteer4.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/CurrentToTorque"))
  {
    n.getParam("Steer4/CurrentToTorque", m_MotorSteer4.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/CurrMax"))
  {
    n.getParam("Steer4/CurrMax", m_MotorSteer4.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/CurrMax"))
  {
    n.getParam("Steer4/CurrMax", m_MotorSteer4.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/HomingDigIn"))
  {
    n.getParam("Steer4/HomingDigIn", m_MotorSteer4.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/HomingTimeout"))
  {
    n.getParam("Steer4/HomingTimeout", m_MotorSteer4.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Steer4/Modulo"))
  {
    n.getParam("Steer4/Modulo", m_MotorSteer4.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Steer4-Modulo parameter from parameter server");
    return 1;
  }

//---------------------drive4------------------------------------------------------------------------
  //loading drive params

  if (n.hasParam("CanOpenIDs/TxPDO1_W4Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO1_W4Drive", m_MotorDrive4.iTxPDO1);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO1_W4Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxPDO2_W4Drive"))
  {
    n.getParam("CanOpenIDs/TxPDO2_W4Drive", m_MotorDrive4.iTxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxPDO2_W4Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxPDO2_W4Drive"))
  {
    n.getParam("CanOpenIDs/RxPDO2_W4Drive", m_MotorDrive4.iRxPDO2);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxPDO2_W4Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/TxSDO_W4Drive"))
  {
    n.getParam("CanOpenIDs/TxSDO_W4Drive", m_MotorDrive4.iTxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-TxSDO_W4Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("CanOpenIDs/RxSDO_W4Drive"))
  {
    n.getParam("CanOpenIDs/RxSDO_W4Drive", m_MotorDrive4.iRxSDO);
  }
  else   
  {
    ROS_ERROR("FAILED to load CanOpenID-RxSDO_W4Drive parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/EncIncrPerRevMot"))
  {
    n.getParam("Drive4/EncIncrPerRevMot", m_MotorDrive4.iEncIncrPerRevMot);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-EncIncrPerRevMot parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/VelMeasFrqHz"))
  {
    n.getParam("Drive4/VelMeasFrqHz", m_MotorDrive4.dVelMeasFrqHz);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-VelMeasFrqHz parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/BeltRatio"))
  {
    n.getParam("Drive4/BeltRatio", m_MotorDrive4.dBeltRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-BeltRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/GearRatio"))
  {
    n.getParam("Drive4/GearRatio", m_MotorDrive4.dGearRatio);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-GearRatio parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/Sign"))
  {
    n.getParam("Drive4/Sign", m_MotorDrive4.iSign);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-Sign parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/VelMaxEncIncrS"))
  {
    n.getParam("Drive4/VelMaxEncIncrS", m_MotorDrive4.dVelMaxEncIncrS);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-VelMaxEncIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/AccIncrS"))
  {
    n.getParam("Drive4/AccIncrS", m_MotorDrive4.dAccIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-AccIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/DecIncrS"))
  {
    n.getParam("Drive4/DecIncrS", m_MotorDrive4.dDecIncrS2);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-DecIncrS parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/EncOffsetIncr"))
  {
    n.getParam("Drive4/EncOffsetIncr", m_MotorDrive4.iEncOffsetIncr);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-EncOffsetIncr parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/IsSteering"))
  {
    n.getParam("Drive4/IsSteering", m_MotorDrive4.bIsSteer);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-IsSteering parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/CurrentToTorque"))
  {
    n.getParam("Drive4/CurrentToTorque", m_MotorDrive4.dCurrentToTorque);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-CurrentToTorque parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/CurrMax"))
  {
    n.getParam("Drive4/CurrMax", m_MotorDrive4.dCurrMax);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-CurrMax parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/HomingDigIn"))
  {
    n.getParam("Drive4/HomingDigIn", m_MotorDrive4.iHomingDigIn);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-HomingDigIn parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/HomingTimeout"))
  {
    n.getParam("Drive4/HomingTimeout", m_MotorDrive4.iHomingTimeout);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-HomingTimeout parameter from parameter server");
    return 1;
  }


  if (n.hasParam("Drive4/Modulo"))
  {
    n.getParam("Drive4/Modulo", m_MotorDrive4.iModulo);
  }
  else   
  {
    ROS_ERROR("FAILED to load Drive4-Modulo parameter from parameter server");
    return 1;
  }



//------------------------------------------------------------------------------------------------------------


 return 0;
}



int driveErrors(int iErrorNum)
{
  if((iErrorNum==m_DriveError.iOverHeating) || (iErrorNum==m_DriveError.iOverVoltage) || (iErrorNum==m_DriveError.iUnderVoltage) || (iErrorNum==m_DriveError.iMotorOff) || 
     (iErrorNum==m_DriveError.iCurrentLimintOn) || (iErrorNum==m_DriveError.iFeedbackLoss) || (iErrorNum==m_DriveError.iPeakCurrentExced) || (iErrorNum==m_DriveError.iSpeedTrack) ||
     (iErrorNum==m_DriveError.iPositionTrack) || (iErrorNum==m_DriveError.iSpeedLimit) || (iErrorNum==m_DriveError.iMotorStuck) )
  {
    return 1;
  }

  else if(iErrorNum==m_DriveError.iShortCircuit)
  {
    return -1;
  }

  return 0;
}


void displayErrors(int iErrorNum)
{
  if(iErrorNum==m_DriveError.iInitPosNotSet)
  {
    ROS_ERROR("Intitial positon is not set");
  }

  else if(iErrorNum==m_DriveError.iStatusReqFail)
  {
    ROS_ERROR("No answer on status request");
  }


  else if(iErrorNum==m_DriveError.iOverHeating)
  {
    ROS_ERROR("Over heating");
    ROS_INFO("The environment is too hot, or lacks heat removal or there may be a large thermal resistance between the drive and its mounting.");
  }

  else if(iErrorNum==m_DriveError.iShortCircuit)
  {
    ROS_ERROR("Drive error short cirucit");
    ROS_INFO("The motor or its wiring may be defective.");
  }

  else if(iErrorNum==m_DriveError.iOverVoltage)
  {
    ROS_ERROR("Drive error over voltage");
    ROS_INFO("The power supply voltage is too large, or the servo drive did not succeed in absorbing the kinetic energy while braking a load. A shunt resistor may be needed.");
  }

  else if(iErrorNum==m_DriveError.iUnderVoltage)
  {
    ROS_ERROR("Drive error under voltage");
    ROS_INFO("The power supply is shut off or it has too high an impedance.");
  }

  else if(iErrorNum==m_DriveError.iMotorOff)
  {
    ROS_ERROR("Motor is still Off");
  }

  else if(iErrorNum==m_DriveError.iCurrentLimintOn)
  {
    ROS_ERROR("Motor current limit on");
  }

  else if(iErrorNum==m_DriveError.iFeedbackLoss)
  {
    ROS_ERROR("feedback loss");
    ROS_INFO("No match between encoder and Hall location.Available in encoder + Hall feedback systems..");
  }

  else if(iErrorNum==m_DriveError.iPeakCurrentExced)
  {
    ROS_ERROR("Peak current excced");
    ROS_INFO("Possible reasons are drive malfunction or bad tuning of the current controller.");
  }

  else if(iErrorNum==m_DriveError.iSpeedTrack)
  {
    ROS_ERROR("Speed track error");
    ROS_INFO("Bad tuning of the speed controller (or)  Too tight a speed error tolerance (or) Inability of motor to accelerate to the required speed due to too low a line voltage or not a powerful enough motor");
  }

  else if(iErrorNum==m_DriveError.iPositionTrack)
  {
    ROS_ERROR("position track error");
    ROS_INFO("Bad tuning of the position or speed controller (or) Too tight a position error tolerance (or) Abnormal motor load, or reaching a mechanical limit");
  }

  else if(iErrorNum==m_DriveError.iSpeedLimit)
  {
    ROS_ERROR("speed limit exceeded");
    ROS_INFO("speed has exceedded the limits");
  }
 
  else if(iErrorNum==m_DriveError.iMotorStuck)
  {
    ROS_ERROR("motor stuck");
    ROS_INFO("A stuck motor is a motor that does not respond to the applied current command, due to failure of the motor, the drive system or the motion sensor.");
  }

}



bool srvCallback_Homing(neo_kinematics_omnidrive::Homing::Request  &req, neo_kinematics_omnidrive::Homing::Response &res )
{
  bService_called=true;                         //bService_called stores the boolean used this to make stop function execuate only once
  
  res.success.data=true;
  
  return true;
}
bool er(double no, double pos)
{

    bool homing = true;
    double dDeltaPhi = 0.0 - pos;
    // std::cout<<dDeltaPhi<<","<<no<<std::endl;
    if (fabs(dDeltaPhi) < 0.042) //alter Wert=0.03
      {
      dDeltaPhi = 0;
      } 
    else
      {
      homing = false;
      }
    double dVelCmd = 0.85 * dDeltaPhi;

    if(no == 1)
    {DM1.setVelInRadS(1, dVelCmd);
    }
    else if(no == 2)
    {DM2.setVelInRadS(1, dVelCmd);}
    else if(no == 3)
    {DM3.setVelInRadS(1, dVelCmd);} 
    else if(no == 4)
    {DM4.setVelInRadS(1, dVelCmd);}
    return homing;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NeoKinOmnidrive");                    //initialize ros node  
  ros::NodeHandle n;                                           //ros node handle
  std::vector <int> viRet,viRet1,viRet2,viRet3,viRet4;                                     //vector that stores the return value of recMessages() function
  int iTimeElapsed,iTimeSleep;                                 //variable to store elapsed time
  std::chrono::steady_clock::time_point aStart,aStartTime, aStartTime1;     //aStart stores the start time for homing
  int iNumOfMotors=2;                                          //total no of motors
  bool bDriveError;                                            //boolean variable that tells presence of drive error
  //creating a vectors to store positoin and velocity of steer and drive motors of DM1
  std::vector<double> vdPosGearRad,vdVelGearRadS, vdTorGear;
  vdPosGearRad.resize(2,0);
  vdVelGearRadS.resize(2,0);
  vdTorGear.resize(2,0);


  

  //loading all the required params from yaml file
  if(!(loadingParams(n) == 0))
  {
    ROS_ERROR("Error occured while loading the params");
    return -1;
  }
 
   m_iDriveState=ST_DRIVE_NOT_INIT;

   //declaring variable to store ther return value of init funciton
   int iErrorVal1,iErrorVal2,iErrorVal3,iErrorVal4;



   // starting the canopen network using network mangaement protocol
   DM1.sendNetStartCanOpen();


   //function which initializes the drive module and turns on motor
   iErrorVal1= DM1.init(
                       m_MotorSteer1.iTxPDO1,m_MotorSteer1.iTxPDO2,m_MotorSteer1.iRxPDO2,m_MotorSteer1.iTxSDO,m_MotorSteer1.iRxSDO,m_MotorSteer1.iEncIncrPerRevMot,
                       m_MotorSteer1.dVelMeasFrqHz, m_MotorSteer1.dBeltRatio, m_MotorSteer1.dGearRatio,  m_MotorSteer1.iSign,  m_MotorSteer1.dVelMaxEncIncrS,
                       m_MotorSteer1.dAccIncrS2,  m_MotorSteer1.dDecIncrS2 ,m_MotorSteer1.iEncOffsetIncr, m_MotorSteer1.bIsSteer, m_MotorSteer1.dCurrentToTorque,
                       m_MotorSteer1.dCurrMax, m_MotorSteer1.iHomingDigIn,m_MotorSteer1.iHomingTimeout,m_MotorSteer1.iModulo,
                       m_MotorDrive1.iTxPDO1,m_MotorDrive1.iTxPDO2,m_MotorDrive1.iRxPDO2,m_MotorDrive1.iTxSDO,m_MotorDrive1.iRxSDO, m_MotorDrive1.iEncIncrPerRevMot,
                       m_MotorDrive1.dVelMeasFrqHz, m_MotorDrive1.dBeltRatio, m_MotorDrive1.dGearRatio,  m_MotorDrive1.iSign, m_MotorDrive1.dVelMaxEncIncrS,
                       m_MotorDrive1.dAccIncrS2 ,m_MotorDrive1.dDecIncrS2, m_MotorDrive1.iEncOffsetIncr, m_MotorDrive1.bIsSteer, m_MotorDrive1.dCurrentToTorque,
                       m_MotorDrive1.dCurrMax,m_MotorDrive1.iHomingDigIn, m_MotorDrive1.iHomingTimeout,m_MotorDrive1.iModulo
                      );

   iErrorVal2= DM2.init(
                       m_MotorSteer2.iTxPDO1,m_MotorSteer2.iTxPDO2,m_MotorSteer2.iRxPDO2,m_MotorSteer2.iTxSDO,m_MotorSteer2.iRxSDO,m_MotorSteer2.iEncIncrPerRevMot,
                       m_MotorSteer2.dVelMeasFrqHz, m_MotorSteer2.dBeltRatio, m_MotorSteer2.dGearRatio,  m_MotorSteer2.iSign,  m_MotorSteer2.dVelMaxEncIncrS,
                       m_MotorSteer2.dAccIncrS2,  m_MotorSteer2.dDecIncrS2 ,m_MotorSteer2.iEncOffsetIncr, m_MotorSteer2.bIsSteer, m_MotorSteer2.dCurrentToTorque,
                       m_MotorSteer2.dCurrMax, m_MotorSteer2.iHomingDigIn,m_MotorSteer2.iHomingTimeout,m_MotorSteer2.iModulo,
                       m_MotorDrive2.iTxPDO1,m_MotorDrive2.iTxPDO2,m_MotorDrive2.iRxPDO2,m_MotorDrive2.iTxSDO,m_MotorDrive2.iRxSDO, m_MotorDrive2.iEncIncrPerRevMot,
                       m_MotorDrive2.dVelMeasFrqHz, m_MotorDrive2.dBeltRatio, m_MotorDrive2.dGearRatio,  m_MotorDrive2.iSign, m_MotorDrive2.dVelMaxEncIncrS,
                       m_MotorDrive2.dAccIncrS2 ,m_MotorDrive2.dDecIncrS2, m_MotorDrive2.iEncOffsetIncr, m_MotorDrive2.bIsSteer, m_MotorDrive2.dCurrentToTorque,
                       m_MotorDrive2.dCurrMax,m_MotorDrive2.iHomingDigIn, m_MotorDrive2.iHomingTimeout,m_MotorDrive2.iModulo
                      );

   iErrorVal3= DM3.init(
                       m_MotorSteer3.iTxPDO1,m_MotorSteer3.iTxPDO2,m_MotorSteer3.iRxPDO2,m_MotorSteer3.iTxSDO,m_MotorSteer3.iRxSDO,m_MotorSteer3.iEncIncrPerRevMot,
                       m_MotorSteer3.dVelMeasFrqHz, m_MotorSteer3.dBeltRatio, m_MotorSteer3.dGearRatio,  m_MotorSteer3.iSign,  m_MotorSteer3.dVelMaxEncIncrS,
                       m_MotorSteer3.dAccIncrS2,  m_MotorSteer3.dDecIncrS2 ,m_MotorSteer3.iEncOffsetIncr, m_MotorSteer3.bIsSteer, m_MotorSteer3.dCurrentToTorque,
                       m_MotorSteer3.dCurrMax, m_MotorSteer3.iHomingDigIn,m_MotorSteer3.iHomingTimeout,m_MotorSteer3.iModulo,
                       m_MotorDrive3.iTxPDO1,m_MotorDrive3.iTxPDO2,m_MotorDrive3.iRxPDO2,m_MotorDrive3.iTxSDO,m_MotorDrive3.iRxSDO, m_MotorDrive3.iEncIncrPerRevMot,
                       m_MotorDrive3.dVelMeasFrqHz, m_MotorDrive3.dBeltRatio, m_MotorDrive3.dGearRatio,  m_MotorDrive3.iSign, m_MotorDrive3.dVelMaxEncIncrS,
                       m_MotorDrive3.dAccIncrS2 ,m_MotorDrive3.dDecIncrS2, m_MotorDrive3.iEncOffsetIncr, m_MotorDrive3.bIsSteer, m_MotorDrive3.dCurrentToTorque,
                       m_MotorDrive3.dCurrMax,m_MotorDrive3.iHomingDigIn, m_MotorDrive3.iHomingTimeout,m_MotorDrive3.iModulo
                      );

   iErrorVal4= DM4.init(
                       m_MotorSteer4.iTxPDO1,m_MotorSteer4.iTxPDO2,m_MotorSteer4.iRxPDO2,m_MotorSteer4.iTxSDO,m_MotorSteer4.iRxSDO,m_MotorSteer4.iEncIncrPerRevMot,
                       m_MotorSteer4.dVelMeasFrqHz, m_MotorSteer4.dBeltRatio, m_MotorSteer4.dGearRatio,  m_MotorSteer4.iSign,  m_MotorSteer4.dVelMaxEncIncrS,
                       m_MotorSteer4.dAccIncrS2,  m_MotorSteer4.dDecIncrS2 ,m_MotorSteer4.iEncOffsetIncr, m_MotorSteer4.bIsSteer, m_MotorSteer4.dCurrentToTorque,
                       m_MotorSteer4.dCurrMax, m_MotorSteer4.iHomingDigIn,m_MotorSteer4.iHomingTimeout,m_MotorSteer4.iModulo,
                       m_MotorDrive4.iTxPDO1,m_MotorDrive4.iTxPDO2,m_MotorDrive4.iRxPDO2,m_MotorDrive4.iTxSDO,m_MotorDrive4.iRxSDO, m_MotorDrive4.iEncIncrPerRevMot,
                       m_MotorDrive4.dVelMeasFrqHz, m_MotorDrive4.dBeltRatio, m_MotorDrive4.dGearRatio,  m_MotorDrive4.iSign, m_MotorDrive4.dVelMaxEncIncrS,
                       m_MotorDrive4.dAccIncrS2 ,m_MotorDrive4.dDecIncrS2, m_MotorDrive4.iEncOffsetIncr, m_MotorDrive4.bIsSteer, m_MotorDrive4.dCurrentToTorque,
                       m_MotorDrive4.dCurrMax,m_MotorDrive4.iHomingDigIn, m_MotorDrive4.iHomingTimeout,m_MotorDrive4.iModulo
                      );

   //it displays if there is any error
   displayErrors(iErrorVal1);
   displayErrors(iErrorVal2);
   displayErrors(iErrorVal3);
   displayErrors(iErrorVal4);
   if(iErrorVal1==0 && iErrorVal2==0 && iErrorVal3==0 && iErrorVal4==0)
   {
     m_iDriveState=ST_DRIVE_INIT;
   }
   int state = 1;
   //ros serviceserver for homing
   ros::ServiceServer srvServer_Homing = n.advertiseService("start_homing", srvCallback_Homing);
   ros::Subscriber topicPub_isEmergencyStop = n.subscribe("state", 1000, EmgCB);
   ros::Rate loop_rate(100);
   double m_d0 = 1;
   bool homing = true;
   bool homing1 = true; 
   bool homing2 = true; 
   bool homing3 = true; 

   bool status1;
   bool status2;
   bool status3;
   bool status4;
   bool bhm_done = 0;
   bool bhm_done2 = 0;
   bool bhm_done3 = 0;
   bool bhm_done4 = 0;
   bool f1 = true;
   bool f2,f3,f4;

   while (ros::ok())
   {  
     //receiving the messages and  stores it in vector
      viRet1= DM1.recMessages(); 
      viRet2= DM2.recMessages(); 
      viRet3= DM3.recMessages(); 
      viRet4= DM4.recMessages(); 
      viRet.insert(viRet.begin(), viRet1.begin(), viRet1.end());
      viRet.insert(viRet.end(),viRet2.begin(), viRet2.end());
      viRet.insert(viRet.end(),viRet3.begin(), viRet3.end());
      viRet.insert(viRet.end(),viRet4.begin(), viRet4.end());

      for (int i = 0; i < viRet.size(); i++) 
      {
        //checking if any error present in the received vector by passing vector to the driveErrors function
        if((driveErrors(viRet[i])==1))
        {
          //displayErrors function displays the error obtained
          displayErrors(viRet[i]);
          //storing our current state of drive in variable m_iStoreState
          m_iStoreState=m_iDriveState;
          //in case of errors present changing the state of drive to ST_DRIVE_ERROR
          ROS_INFO("Rectify the above error");
          //chaning it to the state ST_DRIVE_ERROR because of error presence
          m_iDriveState=ST_DRIVE_ERROR;
        }
        
        //checking if any error obtained in vector is critical
        if((driveErrors(viRet[i])==-1))
        {
          displayErrors(viRet[i]);
          //in case of critical errors it exits the program
          return -1;
        }
      
      }
      //if the dirve state is  ST_DRIVE_ERROR
      if(m_iDriveState==ST_DRIVE_ERROR)
      {
        for (int i = 0; i < viRet.size(); i++) 
        {
          //checking if any errors present in received vector by passing it through driveErrors function
          if((driveErrors(viRet[i])==1))
          {
            //in case if any error present changing the boolean value of bDriveError as true
            bDriveError=true;
          }

        }

        if(bDriveError!=true)
        {
          //if no error present then going back to our previous state by retriving it from m_iStoreState variable
          m_iDriveState=m_iStoreState;
        }
      }

      //state Drive initialized
      else if(m_iDriveState==ST_DRIVE_INIT)
      {
        ROS_INFO("Drive initialized succesfully!");
        ROS_INFO("Use ROS Service start_homing to start homing the drives.");
        // state updated as not homed
        m_iDriveState=ST_NOT_HOMED;   
      }

      else if(m_iDriveState==ST_DRIVE_NOT_INIT)
      {
        //state drive not initialized
        ROS_ERROR("Failed to initialize drives, check hardware and CAN bus connection");
        m_iDriveState=ST_NOT_HOMED;
      }



      else if(bService_called==true)
      {
        DM1.stopMotion();                      //stops the robot motion to perform homing
        DM2.stopMotion(); 
        DM3.stopMotion(); 
        DM4.stopMotion();        
        m_iDriveState=ST_SERVICE_CALLED;       //changing the state as ST_SERVICE_CALLED   
        bService_called=false;                 //making the bService_called variable as false
      }

      else if(m_iDriveState == ST_SERVICE_CALLED)
      {
        bool s1 = false,s2=false,s3=false,s4=false;
        // do
        // {
        if(bEMstate != 1)
        {
          s1 = DM4.TriggeredCondition();
          DM4.recMessages();
          s2 = DM3.TriggeredCondition();
          DM3.recMessages();
          s3 = DM2.TriggeredCondition();
          DM2.recMessages();
          s4 = DM1.TriggeredCondition();
          DM1.recMessages();
          if((s1 && s2 && s3 && s4) == true && flag == 0)
          {
          ROS_INFO_ONCE("Avoiding the garbage - Debug");
          flag++;
          }
          
          else if((s1 && s2 && s3 && s4) == true && flag > 2)
            { 
              ROS_INFO_ONCE("Pre homing is done now");
              m_iDriveState = ST_CONFIGURE_HOMING;
            }
          
          else
          {
            flag++;
            // std::cout<<flag<<std::endl;
            ROS_INFO_ONCE("Pre homing is not done - Debug");

        }
          aStartTime = std::chrono::steady_clock::now();}
        else
        {
          ROS_WARN_ONCE("EM active");
          m_iDriveState = ST_EMERGENCY;
        }
        
      }
      
      else if( m_iDriveState==ST_CONFIGURE_HOMING)
      { 
        auto aEndTime = std::chrono::steady_clock::now();
        iTimeSleep = std::chrono::duration_cast<std::chrono::microseconds>( aEndTime - aStartTime ).count();
        if(bEMstate != 1)
        {
        if(iTimeSleep>100000)
          {
            DM1.configureHoming();
            DM2.configureHoming();
            DM3.configureHoming();
            DM4.configureHoming();
            m_iDriveState = ST_ARM_HOMING;
            aStartTime = std::chrono::steady_clock::now();
          }
        }
        else
        {
          ROS_WARN_ONCE("EM active");
          m_iDriveState = ST_EMERGENCY;          
        }
      }

      else if(m_iDriveState == ST_ARM_HOMING)
      {
        auto aEndTime = std::chrono::steady_clock::now();
        iTimeSleep = std::chrono::duration_cast<std::chrono::microseconds>( aEndTime - aStartTime ).count();
        //if time elapsed less than the give time for arm homing
        //waiting for 5 seconds before arming
        if(bEMstate != 1)
        {
        if(iTimeSleep>100000)
          {
            ROS_INFO_ONCE("Arming the wheels");
            aStartTime = std::chrono::steady_clock::now();
            DM1.armHoming();
            DM2.armHoming();
            DM3.armHoming();
            DM4.armHoming();
            m_iDriveState = ST_WAIT_FOR_HOMING;
          }
                }
        else
        {
          ROS_WARN_ONCE("EM active");
          m_iDriveState = ST_EMERGENCY;          
        }

        aStartTime1 = std::chrono::steady_clock::now();

      }
      else if(m_iDriveState == ST_WAIT_FOR_HOMING)
      {
        // Checking for homing status
        
        auto aEndTime = std::chrono::steady_clock::now();
        iTimeSleep = std::chrono::duration_cast<std::chrono::microseconds>( aEndTime - aStartTime1 ).count();
        // Waiting fot the homing to finish
        if(iTimeSleep>100000)
        {
        while(m_iDriveState != ST_RUNNING)
        {
        bool bhm_done = DM1.homingDone();
        bool bhm_done2 = DM2.homingDone();
        bool bhm_done3 = DM3.homingDone(); 
        bool bhm_done4 = DM4.homingDone(); 

        if(bhm_done == 1 )
          {
            status1 = 1;
          }

        if(bhm_done2 == 1 )
          {
            status2 = 1;
          }
   
        if(bhm_done3 == 1 )
          {
            status3 = 1;
          }
        if(bhm_done4 == 1 )
          {
            status4 = 1;
          }
        if(status1*status2*status3*status4 == 1 )
          {
            ROS_INFO_ONCE("Homing signal recieved");
            m_iDriveState = ST_RUNNING;
          // m_iDriveState = ST_DRIVE_INIT;

          }
        }
      }

      }

      else if( m_iDriveState == ST_RUNNING)
      {
        // auto aEndTime = std::chrono::steady_clock::now();
        // iTimeSleep = std::chrono::duration_cast<std::chrono::microseconds>( aEndTime - aStartTime ).count();
        // // Waiting fot the homing to finish
        // if(iTimeSleep>2000000)

        ROS_INFO_ONCE("Errors?");
        aStartTime1 = std::chrono::steady_clock::now();
        
        DM1.getGearPosAndVel(1, &vdPosGearRad[1], &vdVelGearRadS[1]);
        double pos1 = vdPosGearRad[1];
        homing = er(1,pos1);
        DM1.recMessages(); 
        usleep(20000);

        DM2.getGearPosAndVel(1, &vdPosGearRad[1], &vdVelGearRadS[1]);
        double pos2 = vdPosGearRad[1];
        homing1 = er(2,pos2);
        DM2.recMessages(); 
        usleep(20000);

        DM3.getGearPosAndVel(1, &vdPosGearRad[1], &vdVelGearRadS[1]);
        double pos3 = vdPosGearRad[1];
        homing2 = er(3,pos3);
        usleep(20000);

        DM3.recMessages(); 

        DM4.getGearPosAndVel(1, &vdPosGearRad[1], &vdVelGearRadS[1]);
        double pos4 = vdPosGearRad[1];
        homing3 = er(4,pos4);
        usleep(20000);

        DM4.recMessages();

      }
      else if(m_iDriveState == ST_EMERGENCY)
      {
        if(bEMstate == 0)
        {
          m_iDriveState = ST_DRIVE_INIT;
        }
        else
        {
          ROS_WARN_ONCE("Waiting to be cleared");
        }
      }



      loop_rate.sleep();   
      ros::spinOnce();

    }

  return 0;

}
