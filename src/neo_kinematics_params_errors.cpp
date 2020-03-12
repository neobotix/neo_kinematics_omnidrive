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

  // Loading parameters for kinematics 
  if(n.hasParam("timeout"))
  {
    n.getParam("timeout", dTimeout);
    ROS_INFO("Timeout loaded from Parameter-Server is: %fs", dTimeout);
  }
  else
  {
    ROS_ERROR("FAILED to load timeout parameter from parameter server");
  }

  if(n.hasParam("max_trans_velocity"))
  {
    n.getParam("max_trans_velocity", dTransMaxVelocity);
  }
  else
  {
    ROS_ERROR("FAILED to load max_trans_velocity parameter from parameter server");
    return 1;    
  }

  if(n.hasParam("max_rot_velocity"))
  {
    n.getParam("max_rot_velocity", dRadMaxVelocity);
  }
  else
  {
    ROS_ERROR("FAILED to load max_rot_velocity parameter from parameter server");
    return 1;    
  }


  if(n.hasParam("timeout"))
  {
    n.getParam("timeout", dTimeout);
  }
  else
  {
    ROS_ERROR("FAILED to load timeout parameter from parameter server");
    return 1;    
  }

  if(n.hasParam("NumberOfMotors"))
  {
    n.getParam("NumberOfMotors", iNumOfJoints);
  }
  else
  {
    ROS_ERROR("FAILED to load max_rot_velocity parameter from parameter server");
    return 1;    
  }
 
  if (n.hasParam("Geom/DistWheels"))
        {
          n.getParam("Geom/DistWheels", iWheelDistMM);
        }
  else
  {
    ROS_ERROR("FAILED to load wheel distance parameter from parameter server");
    return 1;    
  }

  if (n.hasParam("Geom/RadiusWheel"))
        {
          n.getParam("Geom/RadiusWheel", iWheelRadiusMM);
        }
  else
  {
    ROS_ERROR("FAILED to load wheel radius parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/DistSteerAxisToDriveWheelCenter"))
        {
          n.getParam("Geom/DistSteerAxisToDriveWheelCenter", iSteerAxisDistToDriveWheelMM);
        }
  else
  {
    ROS_ERROR("FAILED to load steer axis distance parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel1XPos"))
        {
          n.getParam("Geom/Wheel1XPos", vdSteerPosWheelXMM[0]);
        }
  else
  {
    ROS_ERROR("FAILED to load steer position of the wheel-1 X parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel1YPos"))
        {
          n.getParam("Geom/Wheel1YPos", vdSteerPosWheelYMM[0]);
        }
  else
  {
    ROS_ERROR("FAILED to load steer position of the wheel-1 Y parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel2XPos"))
        {
          n.getParam("Geom/Wheel2XPos", vdSteerPosWheelXMM[1]);
        }
  else
  {
    ROS_ERROR("FAILED to load steer position of the wheel-2 X parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel2YPos"))
        {
          n.getParam("Geom/Wheel2YPos", vdSteerPosWheelYMM[1]);
        }
   else
  {
    ROS_ERROR("FAILED to load steer position of the wheel-2 Y parameter from parameter server");
    return 1;    
  } 
  if (n.hasParam("Geom/Wheel3XPos"))
        {
          n.getParam("Geom/Wheel3XPos", vdSteerPosWheelXMM[2]);
        }
  else
  {
    ROS_ERROR("FAILED to load wheel-3 X parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel3YPos"))
        {
          n.getParam("Geom/Wheel3YPos", vdSteerPosWheelYMM[2]);
        }
  else
  {
    ROS_ERROR("FAILED to load wheel-3 Y parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel4XPos"))
        {
          n.getParam("Geom/Wheel4XPos", vdSteerPosWheelXMM[3]);
        }
  else
  {
    ROS_ERROR("FAILED to load wheel-4 X parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("Geom/Wheel4YPos"))
        {
          n.getParam("Geom/Wheel4YPos", vdSteerPosWheelYMM[3]);
        }
  else
  {
    ROS_ERROR("FAILED to load wheel-3 Y parameter from parameter server");
    return 1;    
  }  
  //DrivePrms
  
  if (n.hasParam("DrivePrms/MaxDriveRate"))
        {
          n.getParam("DrivePrms/MaxDriveRate", dMaxDriveRadS);
        }
  else
  {
    ROS_ERROR("FAILED to load MaxDriveRate parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("DrivePrms/MaxSteerRate"))
        {
          n.getParam("DrivePrms/MaxSteerRate", dMaxSteerRadS);
        }
  else
  {
    ROS_ERROR("FAILED to load MaxSteerRate parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("DrivePrms/Wheel1NeutralPosition"))
  {
     n.getParam("DrivePrms/Wheel1NeutralPosition", vdWheelNeutralPos[0]);
  }
  else
  {
    ROS_ERROR("FAILED to load Wheel1NeutralPosition parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("DrivePrms/Wheel2NeutralPosition"))
        {
          n.getParam("DrivePrms/Wheel2NeutralPosition", vdWheelNeutralPos[1]);
        }
  else
  {
    ROS_ERROR("FAILED to load Wheel2NeutralPosition parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("DrivePrms/Wheel3NeutralPosition"))
        {
          n.getParam("DrivePrms/Wheel3NeutralPosition", vdWheelNeutralPos[2]);
        }
  else
  {
    ROS_ERROR("FAILED to load Wheel3NeutralPosition parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("DrivePrms/Wheel4NeutralPosition"))
        {
          n.getParam("DrivePrms/Wheel4NeutralPosition", vdWheelNeutralPos[3]);
        }
  else
  {
    ROS_ERROR("FAILED to load Wheel4NeutralPosition parameter from parameter server");
    return 1;    
  }  
  //Thread
  
  if (n.hasParam("Thread/ThrUCarrCycleTimeS"))
        {
          n.getParam("Thread/ThrUCarrCycleTimeS", dCmdRateSec);
        }
  else
  {
    ROS_ERROR("FAILED to load ThrUCarrCycleTimeS parameter from parameter server");
    return 1;    
  }  
  //Motion Control
  
  if (n.hasParam("SteerCtrl/Spring"))
        {
          n.getParam("SteerCtrl/Spring", dSpring);
        }
  else
  {
    ROS_ERROR("FAILED to load Spring parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("SteerCtrl/Damp"))
        {
          n.getParam("SteerCtrl/Damp", dDamp);
        }
  else
  {
    ROS_ERROR("FAILED to load Damp parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("SteerCtrl/VirtMass"))
        {
          n.getParam("SteerCtrl/VirtMass", dVirtualMass);
        }
  else
  {
    ROS_ERROR("FAILED to load VirtMass parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("SteerCtrl/DPhiMax"))
        {
          n.getParam("SteerCtrl/DPhiMax", dDPhiMax);
        }
  else
  {
    ROS_ERROR("FAILED to load DPhiMax parameter from parameter server");
    return 1;    
  }  
  if (n.hasParam("SteerCtrl/DDPhiMax"))
        {
          n.getParam("SteerCtrl/DDPhiMax", dDDPhiMax);
        }
  else
  {
    ROS_ERROR("FAILED to load DDPhiMax parameter from parameter server");
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
