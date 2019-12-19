
#ifndef SOCKETCAN_INCLUDEDEF_H
#define SOCKETCAN_INCLUDEDEF_H

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <neo_kinematics_omnidrive/CanMesg.h>



class SocketCan
{
public:

/*
 *default constructor
 */
  SocketCan();


 /*
  *default constructor
  */
  ~SocketCan();


 /*
  *Initialises the socketcan
  */
  int initSocket();


 /*
  *transmitting the can message
  *@param sCMsg: It is the object for the CanMesg class
  */ 
  void transmitMsg(CanMesg sCMsg);
 

 /*
  *receiving the can message
  *@param sCMsg: It is the object for CanMesg class 
  */
  bool receiveMsg(CanMesg* sCMsg);

private:
  int iSocket;                            //variable to store Socket open
  int iNoBytes;                             //variable that stores no of bytes of data


  struct sockaddr_can addr;               //object for the struct sockaddr_can
  struct can_frame frame;                 //object for the struct can_frame
  struct ifreq ifr;                       //object for the struct ifreq
  const char *ifname = "can0";

};

#endif
