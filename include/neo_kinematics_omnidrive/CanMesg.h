#ifndef CANMESG_INCLUDEDEF_H
#define CANMESG_INCLUDEDEF_H

#include <iostream>

class CanMesg
{
public:
  typedef unsigned char BYTE;

protected:
  BYTE m_Dat[8];

public:
  int m_iId;    // Id of message
  int m_iLen;   // length of message


 /*
  *parameterized constructor
  */
  CanMesg()   
  {
    m_iId=0;
    m_iLen=8;
  }

 /*
  *setting the bytes
  */
  void set(BYTE Data0=0, BYTE Data1=0, BYTE Data2=0, BYTE Data3=0, BYTE Data4=0, BYTE Data5=0, BYTE Data6=0, BYTE Data7=0)
  {
    m_Dat[0]=Data0;
    m_Dat[1] = Data1;
	m_Dat[2] = Data2;
	m_Dat[3] = Data3;
	m_Dat[4] = Data4;
	m_Dat[5] = Data5;
	m_Dat[6] = Data6;
	m_Dat[7] = Data7; 
  }

 /*
  *getting the id of msg
  */
  int getID()
  {
	return m_iId;
  }

 /*
  *getting the length
  */ 
  int getLength()
  {
	return m_iLen;
  }

 /*
  *getting the particular byte
  */

  int getByte(int i)
  {
    return m_Dat[i];
  }



};




#endif
