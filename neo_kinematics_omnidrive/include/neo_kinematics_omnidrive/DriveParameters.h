#ifndef DRIVEPARAMETERS_INCLUDEDEF_H
#define DRIVEPARAMETERS_INCLUDEDEF_H

class DriveParameters
{

public:

/*
 *get max acceleration
 */
double getMaxAccln()
{
  return m_dAccIncrS2;
}


/*
 *get max deceleration
 */
double getMaxDecln()
{
  return m_dDccIncrS2;
}


/*
 *get gear ratio
 */

double getGearratio()
{
  return m_dGearRatio;
}


/*
 *get belt ratio
 */
double getBeltRatio()
{ 
  return m_dBeltRatio;
}


/*
 *get encoder increment per revolution of motor
 */
double getEncoderIncrPerRevOfMot()
{
  return m_iEncoderIncrPerRevOfmotor;
}


/*
 *get direction of motion
 */ 
int getmotion_direction()
{
  return m_iMotionDirection;
}


/*
 *converting velocity gear rad/s to vel motor increament period
 */
int convertVelGearRadSToVelMotIncrPeriod(double dGearvelrads)
{
  return ((int)(dGearvelrads * m_dPositionGearRadToPosMotIncr / m_dVelMeasFrqHz));
}


/*
 *converting encoder increments into gear position in radians
 */
double convertPosMotIncrToPosGearRad(int iPositionCnt)
{
  return ((double)iPositionCnt / m_dPositionGearRadToPosMotIncr);
}


/*
 *conversion of encoder increments to gear vel in rad/s
 */
double convertVelMotIncrPeriodToVelGearRadS(int iVelMotIncrPeriod)
{
  return ((double)iVelMotIncrPeriod/  m_dPositionGearRadToPosMotIncr *m_dVelMeasFrqHz );
}

/*
 *get ident of drives
 */

int getIdentOfDrive()
{
  return m_iIdentofdrive;
}


/*
 *get max vel
 */
double getMaxVel()
{
  return m_dVelMaxEncIncrS;
}


/*
 *get encoder offset
 */
int getEncoderOffset()
{
  return m_iEncOffsetIncrement;
}

//getting digital input for Homing
int getHomDigIn()
{
  return m_iHomDigIn;
}

/*
 *get homing timeout
 */
int getHomTimeOut()
{
  return m_iHomingTimeout;
}


/*
 *getting modulo
 */
int getModulo()
{
  return m_iModulo;
}


/*
 *setting all the params
 */

double getCurrMax()
{
	return m_dCurrMax;
}
void settingParams(
		           int iDriveIdent,
	               int iEncIncrPerRevMot,
		           double dVelMeasFrqHz,
		           double dBeltRatio,
		           double dGearRatio,
		           int iSign,
		           double dVelMaxEncIncrS,
		           double dAccIncrS2,
		           double dDecIncrS2,
		           int iEncOffsetIncr,
		           bool bIsSteer,
                   double dCurrToTorque,
            	   double dCurrMax,
		           int iHomingDigIn,
		           int iHomingTimeout,
		           int iModulo
		          )
{
	m_iIdentofdrive = iDriveIdent;
	m_iEncoderIncrPerRevOfmotor = iEncIncrPerRevMot;
	m_dVelMeasFrqHz = dVelMeasFrqHz;
	m_dBeltRatio = dBeltRatio;
	m_dGearRatio = dGearRatio;
	m_iMotionDirection = iSign;
	m_dVelMaxEncIncrS = dVelMaxEncIncrS;
	m_dAccIncrS2 = dAccIncrS2;
	m_dDccIncrS2 = dDecIncrS2;
	m_iEncOffsetIncrement = iEncOffsetIncr;
	m_bIsSteer = bIsSteer;
    double dPI = 3.14159265358979323846;
    m_dPositionGearRadToPosMotIncr = m_iEncoderIncrPerRevOfmotor * m_dGearRatio
			* m_dBeltRatio / (2. * dPI);

    m_dCurrToTorque = dCurrToTorque;
	m_dCurrMax = dCurrMax;
	m_iHomDigIn = iHomingDigIn;
	m_iHomingTimeout=iHomingTimeout;
	m_iModulo=iModulo;
}

private:

double    m_dAccIncrS2;                                          // max acceleration 
double    m_dDccIncrS2;                                          // max deceleration 
double    m_dGearRatio;                                          // Gear ratio                         
double    m_dBeltRatio;                                          // Belt ratio 
int       m_iEncoderIncrPerRevOfmotor;                           // encoder increments per revolution motor shaft 
int       m_iMotionDirection;                                    // direction of motion
double    m_dVelMaxEncIncrS;                                     // velocity max encoder increments
double    m_dPositionGearRadToPosMotIncr;                        // positon gear rad to pos mot incr      
double    m_dVelMeasFrqHz;                                       // velocity measured in frequecy (hZ)s
int       m_iHomDigIn;                                           // specifies the digital input for homing signal                                         
int       m_iHomingTimeout;                                      // homing timeout        
int       m_iModulo;                                             // modulo
int       m_iIdentofdrive;                                       // identity of drive
int       m_iEncOffsetIncrement;                                 // encoder offset increment
bool      m_bIsSteer;                                            // needed for distinguishing motor while initializing
double    m_dCurrToTorque;                                       // factor to convert motor active current [A] into torque [Nm] 
double    m_dCurrMax;                                            // max current allowed  


};


#endif
