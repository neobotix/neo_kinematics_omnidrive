
#include <neo_kinematics_omnidrive/ElmoMotorCtrl.h>
#include <unistd.h>
#include <neo_kinematics_omnidrive/SocketCan.h>
#include <chrono>
#include <mutex>
using namespace std::chrono;
std::mutex m;


ElmoMotorCtrl::ElmoMotorCtrl()
{
  m_dPositonGearMeasInRad=0;     //Gear positin measurement in radians
  m_dVelGearMeasInRadS=0;        //Gear velocity measurement in rad/s      
  m_iStatus=0;                   //variable to which stores the bit received value of status register command
  // m_sCanCtrl=new SocketCan();
}

ElmoMotorCtrl::~ElmoMotorCtrl()
{
}

void ElmoMotorCtrl::setCanOpenParam( int iTxPDO1, int iTxPDO2, int iRxPDO2, int iTxSDO, int iRxSDO)
{
  //assigning the respective canopen Id's which are loaded from yaml file
  m_ParamCanopen.iTxPDO1 = iTxPDO1;
  m_ParamCanopen.iTxPDO2 = iTxPDO2;
  m_ParamCanopen.iRxPDO2 = iRxPDO2;
  m_ParamCanopen.iTxSDO = iTxSDO;
  m_ParamCanopen.iRxSDO = iRxSDO;

}


void  ElmoMotorCtrl::OpenSocket()
{
   
    m_sCanCtrl.initSocket();
}


int ElmoMotorCtrl::initMotorCtrl()
{
  int iPositionCnt;                                     // posiotion
  bool bPosNotSet=true;                                 // boolean for position is not set
  int iMaxAcc = int(m_DriveParameter.getMaxAccln());    // max accln
  int iMaxDcc = int(m_DriveParameter.getMaxDecln());    // max decln
  int iCount;                                           // counter
  CanMesg Msg;                                          // object for can message  

  //turn of motor
  // setInterpreter(8, 'M', 'O', 0, 0);

  setInterpreter(8, 'X', 'M', 2,m_DriveParameter.getModulo());

  setInterpreter(8, 'X', 'M', 1, -m_DriveParameter.getModulo());


  //setting velocity as motion control( so as per manual following commands should be performed pg-77)
  // setInterpreter(8, 'M', 'O', 0, 0);

  //switch to unit mode 2
  setInterpreter(8, 'U', 'M', 0, 2);
  // switch to profile mode 1(if unit mode 2)
  setInterpreter(8, 'P', 'M', 0, 1);

  // set maximum Acceleration to X Incr/s^2
  setInterpreter(8, 'A', 'C', 0, iMaxAcc);
  // set maximum decceleration to X Incr/s^2
  setInterpreter(8, 'D', 'C', 0, iMaxDcc);


  //set position encounter to zero
  setInterpreter(8, 'P', 'X', 0, 0);

  //getting torque
 

  iCount=0;    //initial counter value
  do
  { 
    //recevies the messages   
    m_sCanCtrl.receiveMsg(&Msg);
    // std::cout<<"eval \n";
    //checking whether initial positon is set
    if(Msg.getByte(0)=='P' && Msg.getByte(1)=='X')
    {
      iPositionCnt=(Msg.getByte(4))|(Msg.getByte(5)<<8)|(Msg.getByte(6)<<16)|(Msg.getByte(7)<<24);
      m_dPositonGearMeasInRad=m_DriveParameter.getmotion_direction()*m_DriveParameter.convertPosMotIncrToPosGearRad(iPositionCnt);
      bPosNotSet=false;
    }
    if (iCount>300)
    {

     bPosNotSet=false;
      // return 1;      // Intitial positon is not set
    } 
    iCount++;


  }while (bPosNotSet==true);


  // ---------- set PDO mapping
  //PDO mapping is done for only TPDO, bcz it enables the drive to send a predefined messsage in response to an event,if it is not confiugred the drives doesnt transmit positon for TPDO1
  // Mapping of TPDO1:
  // - position
  // - velocity
  // stop all emissions of TPDO1(in pg  5-6  in elmo implementation guide)
  sendingSDODownload(0x1A00, 0, 0);

  // position 4 byte of TPDO1
  sendingSDODownload(0x1A00, 1, 0x60640020);

  // velocity 4 byte of TPDO1
  sendingSDODownload(0x1A00, 2, 0x60690020);

  // transmission type "synch"
  sendingSDODownload(0x1800, 2, 1);

  // activate mapped objects
  sendingSDODownload(0x1A00, 0, 2);
  return 0;
}

int ElmoMotorCtrl::configureHoming()
{
  double dHomeVelInRads=-1.0;               //velocity at which steer motor should run while homing
  const int c_iPosReference = m_DriveParameter.getEncoderOffset();   

  //disarm homing
  setInterpreter(8, 'H', 'M', 1, 0);

  //configure homing sequences
  // setting the value sucht that increment counter resets after the homingevent occurs
  setInterpreter(8, 'H', 'M', 2,c_iPosReference);

  //choosing channel/switch on which controller has to listen for change of homing event(high/low/falling/rising)
  setInterpreter(8, 'H', 'M', 3,m_DriveParameter.getHomDigIn());

  //choosing the action the controller should perform afer the homing event occurs
  // HM[4] = 0 : after Event stop immediately
  // HM[4]=2 :do nothing  
  setInterpreter(8, 'H', 'M', 4,0);

  //setting the absolute setting of postion counter {(HM[5]=0) == HM[2]} after the homing event
  setInterpreter(8, 'H', 'M',5,0);

  //turning the motor 
  setVelInRadS(dHomeVelInRads);
  

  return 0;
}


void ElmoMotorCtrl::armHoming()
{
  //arm homing
  setInterpreter(8, 'H', 'M', 1, 1);

}

// int ElmoMotorCtrl::homingDone()
// {
//   //status of homing
//   bool bret = true;
//   CanMesg Msg;
//   m_bLimSwRight == false;
//   setInterpreter(4, 'H', 'M', 1, 0);
//   m_sCanCtrl.receiveMsg(&Msg);
//   if( (Msg.getByte(0) == 'H') && (Msg.getByte(1) == 'M') )
//     { 
//       // status message (homing armed = 1 / disarmed = 0) is encoded in 5th byte
//       if(Msg.getByte(4) == 0)
//       {
//         // if 0 received: elmo disarmed homing after receiving the defined event
//         // std::cout << "Got Homing-Signal "  << std::endl;
//         m_bLimSwRight = true;
//         return 1;  
//       } 
//     }
//   }
bool ElmoMotorCtrl::TriggeredCondition()
{
  // CanMesg Msg;
  // bool bRet;
  

  // Clear the buffer
  // do
  // {
  //   bRet = m_sCanCtrl.receiveMsg(&Msg);
  // }
  // while(bRet == true);

  // Interpreter set
	// std::cout<<m_iDigIn<<std::endl;
  setInterpreter(4, 'I', 'P', 0, 16);
  sendCanMessage(0x80, 0, 0); 

  evaluatingMessageReceived();
  setInterpreter(4,'S','R',0,0);
  sendCanMessage(0x80, 0, 0); 

  evaluatingMessageReceived();
  if( (m_iDigIn & (unsigned int)m_DriveParameter.getHomDigIn())== 0x0000 )
  { 
    setVelInRadS(0.0);
    return true;

  }
  else
  {
    setVelInRadS(0.5);
    return false;
  }
} 



// void ElmoMotorCtrl::armHoming()
// {
//   //arm homing
//   double dHomeVelInRads=-1.0;               //velocity at which steer motor should run while homing

//   //turning the motor 
//   setVelInRadS(dHomeVelInRads);

// }

int ElmoMotorCtrl::homingDone()
{
  //status of homing
	bool bLimitSwitchInActive = false;
	bool bHomingTimeout = false;	
	bool bLoopVariable = true;
  CanMesg Msg;
  bool bRet;
  
	auto aStartTime1 = std::chrono::steady_clock::now();

  // Clear the buffer
  do
  {
    bRet = m_sCanCtrl.receiveMsg(&Msg);
  }
  while(bRet == true);

 // Interpreter set
  // std::cout<<m_iDigIn<<std::endl;

 	setInterpreter(4, 'I', 'P', 0, 16);
	evaluatingMessageReceived();
  setInterpreter(4,'S','R',0,0);
  evaluatingMessageReceived();
  if( (m_iDigIn & (unsigned int)m_DriveParameter.getHomDigIn())== 0x0000 )
  { 
    bLimitSwitchInActive = true;
    // std::cout<<"1"<<std::endl;
    return false;
  }
  else
  {
  	return true;
  }

  auto aEndTime = std::chrono::steady_clock::now();
	auto aEnd = std::chrono::duration_cast<std::chrono::microseconds>( aEndTime - aStartTime1 ).count();

	if(aEnd > 50000)
	{
		bool bHomingTimeout = true;	
		// setVelInRadS(0.0);
		return false;
	}
  // std::cout<<bHomingTimeout;
}


int ElmoMotorCtrl::turnOnMotor()
{
  CanMesg Msg;                   //object for the class CanMesg
  int iCount;                    //declaration of counter value  
  int iStatus;                   //variable to store the bit value which is returned from status request message
  int iReturn;                   //stores the return value from evaluateStatusRegister function
  bool bNoStatusReq=true;        // boolean to tell the whether reply on status request
  bool bRet=true;

  //turning on the motor
  setInterpreter(8,'M','O',0,1);

  //clearing the can buffer
  do
  {
    bRet = m_sCanCtrl.receiveMsg(&Msg);
  }
  while(bRet == true);

  //sending request to evaluate status
  setInterpreter(4,'S','R',0,0);
  iCount=0;
  do
  {
    m_sCanCtrl.receiveMsg(&Msg);
    if(Msg.getByte(0)=='S' && Msg.getByte(1)=='R')
    {
      iStatus=(Msg.getByte(4))|(Msg.getByte(5)<<8)|(Msg.getByte(6)<<16)|(Msg.getByte(7)<<24);
      iReturn = evaluateStatusRegister(iStatus);
      return iReturn;
      bNoStatusReq=false;
    }
    if(iCount>300)
    {
      return 2;        // No answer on status request
      bNoStatusReq=false;
    }
    iCount++;
  }while(bNoStatusReq==true);

  return 0;
}



bool ElmoMotorCtrl::turnOffMotor()
{
  bool bRet = true;
  setInterpreter(8,'M','O',0,0);
  return bRet;
}







void ElmoMotorCtrl::setVelInRadS(double dGearvelrads)
{
  int iVelEncIncrement;   //variable to store the velocity value in encoder increments
  //calculating motor velocit from joint velocity
  iVelEncIncrement=m_DriveParameter.getmotion_direction()*m_DriveParameter.convertVelGearRadSToVelMotIncrPeriod(dGearvelrads);
  //writing conditions such that velocity does not exceed from max ranges
  if(iVelEncIncrement>m_DriveParameter.getMaxVel())
  {
    iVelEncIncrement=(int)m_DriveParameter.getMaxVel();
  }
  else if(iVelEncIncrement<-m_DriveParameter.getMaxVel())
  {
    iVelEncIncrement=-(int)m_DriveParameter.getMaxVel();
  }
  
  //we can configure jog velocity after setting um=2,pm=1 and ac,dc
  setInterpreter(8,'J','V',0,iVelEncIncrement);
  
  //after settomg jog velocity we have to use the begin motion command to run the motor
  setInterpreter(4,'B','G',0,0);

  //sending sync message to trigger devices for sending data
  sendCanMessage(0x80, 0, 0);

  //sending request to evaluate status
  setInterpreter(4,'S','R',0,0);


}


void ElmoMotorCtrl::stopMotion()
{
  CanMesg Msg;                //declaration of object for class CanMesg
  bool bRet=true;             //boolean stores the return value of receiveMsg funciton
  do
  {
    bRet = m_sCanCtrl.receiveMsg(&Msg);
  }
  while(bRet == true);
  //To stop the motion of motor
  setInterpreter(4,'S','T',0,0);
  
  //sending sync message to trigger devices for sending data
  // sendCanMessage(0x80, 0, 0);
}

int ElmoMotorCtrl::evaluateStatusRegister(int iStatus)
{
 
  if(isBitSet(iStatus,6))
  {
     // requesting the detailed description of motor failure
     setInterpreter(4,'M','F',0,0);
  }

  else if(isBitSet(iStatus,0))
  {
    if((0x0000000E & iStatus)==12)
      return 3;   // Over heating

    if((0x0000000E&iStatus)==10)
      return 4;    // Drive error short cirucit

    if((0x0000000E&iStatus)==4)
      return 5;     // Drive error over voltage 
      
    if((0x0000000E&iStatus)==2)
      return 6;     // Drive error under voltage

    setInterpreter(4,'M','F',0,0);      ///sends the status of motor failue message
  }

  else
  {
    if(isBitSet(iStatus,4))
    {    
       return 0;     // no error
    }   

    else
    {   
       return 7;    // Motor is still Off
    }

    if(isBitSet(iStatus,13))
    {
       return 8;    // Motor current limit on
    }
  }
  return 0;
}




std::vector <int> ElmoMotorCtrl::evaluatingMessageReceived()
{
  CanMesg Msg;          //declaration of object for CanMesg  
  int iDigIn;           //declartion for digital input
  int iFailure;         //declaration to store motor failure message bit 
  bool bRet=true;       //declaration to store return value
  int iPosIncrPeriod;   //encoder increments per measurment period for positon
  int iVelIncrPeriod;   //encoder increments per measurement period for velocity
  int iReturn;          //stores the return value of evaluateStatusRegister function


  //sending sync message to trigger devices for sending data
  std::vector <int> viReceivedMsg;
  // sendCanMessage(0x80, 0, 0); 

  // std::cout<<"eval-msg \n";
  // m.lock();
 
    //----------------------------------------------------------------------------------------------------------------------------------
   do
  {   

    bRet=m_sCanCtrl.receiveMsg(&Msg);
    //----------------------------------------------------------------------------------------------------------------------------------

    if(bRet==true)
    {
      //evalutaing from binary interpreter
      if(Msg.m_iId==m_ParamCanopen.iTxPDO2)
      {

        // status register
        if( (Msg.getByte(0) == 'S') && (Msg.getByte(1) == 'R') ) 
        {
          m_iStatus = (Msg.getByte(7) << 24) | (Msg.getByte(6) << 16)| (Msg.getByte(5) << 8) | (Msg.getByte(4) );
          iReturn=evaluateStatusRegister(m_iStatus);
          viReceivedMsg.push_back(iReturn);
        }
        // motor failure
        else if( (Msg.getByte(0) == 'M') && (Msg.getByte(1) == 'F') ) 
        {
          iFailure = (Msg.getByte(7) << 24) | (Msg.getByte(6) << 16)| (Msg.getByte(5) << 8) | (Msg.getByte(4) );
          if( isBitSet(iFailure, 2) )
          {
            viReceivedMsg.push_back(9);          // feedback loss
          }

          if( isBitSet(iFailure, 3) )
          {
            viReceivedMsg.push_back(10);        // Peak current excced
          }

          if( isBitSet(iFailure, 7) )
          {
            viReceivedMsg.push_back(11);        // Speed track error
          }

          if( isBitSet(iFailure, 8) )
          {
            viReceivedMsg.push_back(12);       // position track error
          }

          if( isBitSet(iFailure, 17) )
          {
            viReceivedMsg.push_back(13);       // speed limit exceeded
          }

          if( isBitSet(iFailure, 21) )
          {
            viReceivedMsg.push_back(14);        // motor stuck
          }
        }
        else if( (Msg.getByte(0) == 'H') && (Msg.getByte(1) == 'M') )
        {
          // status message (homingDone armed = 1 / disarmed = 0) is encoded in 5th byte
          if(Msg.getByte(4) == 0)
          {
          // if 0 received: elmo disarmed homingDone after receiving the defined event
            viReceivedMsg.push_back(DISARM);  //elmo disarmed after homing 
          }
        }
        else if( (Msg.getByte(0) == 'I') && (Msg.getByte(1) == 'Q') )
        {
          int iVal=0;
          iVal = (Msg.getByte(7) << 24) | (Msg.getByte(6) << 16)
            | (Msg.getByte(5) << 8) | (Msg.getByte(4) );
          float* pfVal;
          pfVal=(float*)&iVal;
          m_dMotorCurr = *pfVal;      
         }
         else if ( ( Msg.getByte ( 0 ) == 'I' ) && ( Msg.getByte ( 1 ) == 'P' ) ) // digital in == limit switches
        {
          m_iDigIn = 0x1FFFFF & ( ( Msg.getByte ( 7 ) << 24 ) | ( Msg.getByte ( 6 ) << 16 )
                                  | ( Msg.getByte ( 5 ) << 8 ) | ( Msg.getByte ( 4 ) ) );
        }

      }

 
      //--------------------------------------------------------------------------------------------------------------------
      // evaluate messages from TPD01 which is transmitted on sync msg
      if(Msg.m_iId==m_ParamCanopen.iTxPDO1)
      {
        //gets the position encoder increments per period
        iPosIncrPeriod = (Msg.getByte(3) << 24) | (Msg.getByte(2) << 16)| (Msg.getByte(1) << 8) | (Msg.getByte(0) );

        m_dPositonGearMeasInRad=m_DriveParameter.getmotion_direction()* m_DriveParameter.convertPosMotIncrToPosGearRad(iPosIncrPeriod);

        //gets the velocity encoder increments per period

        iVelIncrPeriod = (Msg.getByte(7) << 24) | (Msg.getByte(6) << 16)| (Msg.getByte(5) << 8) | (Msg.getByte(4) );

        m_dVelGearMeasInRadS=m_DriveParameter.getmotion_direction() * m_DriveParameter.convertVelMotIncrPeriodToVelGearRadS(iVelIncrPeriod);

        viReceivedMsg.push_back(0);

      } 

    }

  }while(bRet==true);

// m.unlock();

  return viReceivedMsg;
}



void ElmoMotorCtrl::getGearPosAndVel(double *pdPositonGearMeasInRad, double *pdVelGearMeasInRadS)
{
  //gets the positon and velocity 
  *pdPositonGearMeasInRad = m_dPositonGearMeasInRad;
  *pdVelGearMeasInRadS    = m_dVelGearMeasInRadS;
}


void ElmoMotorCtrl::getGearVel(double *pdVelGearMeasInRadS)
{
  //gets the velocity
  *pdVelGearMeasInRadS  = m_dVelGearMeasInRadS;
}

void ElmoMotorCtrl::getGearTor(double *pdTorGear)
{
  //gets the Torque
  setInterpreter(4, 'I', 'Q', 0, 0);  // active current

  *pdTorGear = m_dMotorCurr * dTorqueConstant;
}

void ElmoMotorCtrl::setGearTor(double dTorqueNm)
{
  float fMotCurr = dTorqueNm / dTorqueConstant;
  if  (fMotCurr > m_DriveParameter.getCurrMax())
    {
      fMotCurr = m_DriveParameter.getCurrMax();
      std::cout << "Torque command too high: " << fMotCurr << " Nm. Torque has been limitited." << std::endl;
    }
  if (fMotCurr < -m_DriveParameter.getCurrMax())
    {
      fMotCurr = -m_DriveParameter.getCurrMax();
      std::cout << "Torque command too high: " << fMotCurr << " Nm. Torque has been limitited." << std::endl;
    }
  std::cout<<fMotCurr;

  setInterpreter(8, 'T', 'C', 0, fMotCurr);

  // setInterpreter(4,'B','G',0,0);

  // sendCanMessage(0x80, 0, 0);



  //sending request to evaluate status
  // setInterpreter(4,'S','R',0,0);
}

bool ElmoMotorCtrl::isBitSet(int iValue, int iBit)
{
  //checks is bit set
  if((iValue & (1<<iBit))==0)
    return false;
  else
    return true;
}



void ElmoMotorCtrl::sendingSDODownload(int iIndex, int iSubindex, int iData)
{
  CanMesg msg;                       //object declaration for class CanMesg
  msg.m_iId=m_ParamCanopen.iRxSDO;   //message I'd
  msg.m_iLen=8;                      //message length
 
  const int ciInitDownloadRequest = 0x20;//for bits 5,6,7 of byte 0  should have 001(pg4-2 in implementation guide)
  const int ciByteswithNoData = 0x00; //  for bits 2,3,4 should have 000
  const int ciExpedited = 0x02;//for bit 1 should have 1
  const int ciDataSizeIndicator = 0x01;//for bit 0 shoud have 1

  unsigned char cMesg[8];

  cMesg[0]=ciInitDownloadRequest|ciExpedited|ciDataSizeIndicator|(ciByteswithNoData<<2);
  cMesg[1]=iIndex;
  cMesg[2]=iIndex>>8;
  cMesg[3]=iSubindex;
  cMesg[4]=iData;
  cMesg[5]=iData>>8;
  cMesg[6]=iData>>16;
  cMesg[7]=iData>>24;

  msg.set(cMesg[0],cMesg[1],cMesg[2],cMesg[3],cMesg[4],cMesg[5],cMesg[6],cMesg[7]);
  m_sCanCtrl.transmitMsg(msg);
}







void ElmoMotorCtrl::setInterpreter(int iDatalen,char cmdchar1,char cmdchar2,int iIndex,int iData )
{
  char cIndex[2];
  char cInt[4];
  CanMesg Cmsg;                            //object declaration for class CanMesg
  Cmsg.m_iId=m_ParamCanopen.iRxPDO2;       //message I'd
  Cmsg.m_iLen=iDatalen;                    //message length

  cIndex[0]=iIndex;
  cIndex[1]=(iIndex >> 8) & 0x3F;

  cInt[0] = iData;
  cInt[1] = iData >> 8;
  cInt[2] = iData >> 16;
  cInt[3] = iData >> 24;

  Cmsg.set(cmdchar1,cmdchar2,cIndex[0],cIndex[1],cInt[0],cInt[1],cInt[2],cInt[3]);
  m_sCanCtrl.transmitMsg(Cmsg);

}




void ElmoMotorCtrl::sendCanMessage(int iId, int iLen, unsigned char cByte)
{
  // std::cout<<"sync msg \n";
  CanMesg msg;                   //object declaration for class CanMesg
  msg.m_iId=iId;                 //message I'd
  msg.m_iLen=iLen;               //message length
  unsigned char cMesg[8];

  cMesg[0]=cByte;
  cMesg[1]=0;
  cMesg[2]=0;
  cMesg[3]=0;
  cMesg[4]=0;
  cMesg[5]=0;
  cMesg[6]=0;
  cMesg[7]=0;

  msg.set(cMesg[0],cMesg[1],cMesg[2],cMesg[3],cMesg[4],cMesg[5],cMesg[6],cMesg[7]);
  m_sCanCtrl.transmitMsg(msg);
}


















/*bool ElmoMotorCtrl::Watchdog(bool bBegin)
{
  if(bBegin==true)
  {

    const int c_iHeartbeatTimeMS = 1000;
    const int c_iNMTNodeID = 0x00;
    m_Watchdog=true;


    //heart beat consumer

    sendingSDODownload(0x1016, 1, (c_iNMTNodeID << 16) | c_iHeartbeatTimeMS);

    //error modes after failure : is 0=pre-operational, 1=no state change, 2=stopped
    sendingSDODownload(0x1029, 1, 2);

    //motor behaviour after heart beat failure : quick stopped
    sendingSDODownload(0x6007, 0, 3);

    //Object 0x2F21 = "Emergency Events" which cause an Emergency Message,,, Bit 3 is responsible for Heartbeart-Failure.--> Hex 0x08
     sendingSDODownload(0x2F21, 0, 0x08);
     //usleep(20000);
  }

  else
  {
    m_Watchdog=false;
    //motor behaviour :no action
    sendingSDODownload(0x6007, 0, 0);

    //error state: no state change
    sendingSDODownload(0x1029, 1, 1);

    sendingSDODownload(0x2F21, 0, 0x00);
    //usleep(24000);

  }
  return true;
}*/

