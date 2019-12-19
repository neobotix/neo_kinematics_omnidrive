
#include "ros/ros.h"
#include <neo_kinematics_omnidrive/SocketCan.h>

SocketCan::SocketCan()
{
  initSocket(); //initializing the socket can
}

SocketCan::~SocketCan()
{

}

int SocketCan::initSocket()
{
  iSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);                    //opening the socket
  strcpy(ifr.ifr_name, ifname);
  ioctl(iSocket, SIOCGIFINDEX, &ifr);
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(iSocket, (struct sockaddr *)&addr, sizeof(addr));         //binding the socket

  return 0;
}


void SocketCan::transmitMsg(CanMesg sCMsg)
{
  fd_set fds;                                                    //array of bits,one bit for one socket
  struct timeval timeout = {0,10};                               //timeout interval
  int iSel;                                                      //variabĺe to store select function
  FD_ZERO(&fds);                                                 //zero all the bits of array
  FD_SET(iSocket, &fds);
  iSel = select(iSocket+1, NULL, &fds, NULL, &timeout);          //select func with read func creates a timeout

  if(iSel>0)
  {
    frame.can_id=sCMsg.m_iId;
    frame.can_dlc=sCMsg.m_iLen;

    for(int i=0; i<8; i++)
    {
      frame.data[i] = sCMsg.getByte(i);
    }

    iNoBytes = write(iSocket, &frame, sizeof(struct can_frame));
  }

}



bool SocketCan::receiveMsg(CanMesg* sCMsg)
{
  bool bRet=false;                                              //boolean to return 
  int iSel;                                                     // variabĺe to store select function
  struct timeval timeout = {0,10};                              //timeout interval
  fd_set readSet;                                               //array of bits,one bit for one socket
  FD_ZERO(&readSet);                                            //zero all the bits of array
  FD_SET(iSocket, &readSet);
  iSel=select((iSocket+1 ), &readSet, NULL, NULL, &timeout);    //select func with read func creates a timeout
  if (iSel<=0)
  {
    sCMsg->m_iId=0;
    sCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
          
  }
  else
  {
    iNoBytes = read(iSocket, &frame, sizeof(struct can_frame));
    sCMsg->m_iId =frame.can_id;
    sCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3],frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    /*ROS_INFO("can_id: %X data length: %d data: ", frame.can_id,frame.can_dlc);

    for (int i = 0; i < frame.can_dlc; i++)
        ROS_INFO("%02X ", frame.data[i]);*/
    bRet=true;
  }

  return bRet;
}










  
