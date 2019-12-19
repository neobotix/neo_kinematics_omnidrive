#ifndef NEOOMNIDRIVEERRORS_INCLUDEDEF_H
#define NEOOMNIDRIVEERRORS_INCLUDEDEF_H


/*
 * struct which stores all the errors 
 */

struct DriveErrors
{
  int iNoError=0;                                     // No error             
  int iInitPosNotSet=1;                               // Intitial positon is not set                     
  int iStatusReqFail=2;                               // No answer on status request
  int iOverHeating=3;                                 // Over heating
  int iShortCircuit=4;                                // Drive error short cirucit
  int iOverVoltage=5;                                 // Drive error over voltage 
  int iUnderVoltage=6;                                // Drive error under voltage
  int iMotorOff=7;                                    // Motor is still Off
  int iCurrentLimintOn=8;                             // Motor current limit on
  int iFeedbackLoss=9;                                // feedback loss
  int iPeakCurrentExced=10;                           // Peak current excced
  int iSpeedTrack=11;                                 // Speed track error
  int iPositionTrack=12;                              // position track error
  int iSpeedLimit=13;                                 // speed limit exceeded
  int iMotorStuck=14;                                 // motor stuck

};


//declaring objects for the motor params struct
DriveErrors m_DriveError;


#endif
