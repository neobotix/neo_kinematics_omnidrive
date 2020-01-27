#include <neo_kinematics_omnidrive/NeoKinematics.h>



NeoKinematics::NeoKinematics()
{
	m_vdSteerWheelDistMM.assign(4,0);
	m_vdSteerWheelAngRad.assign(4,0);
	m_vdWheelPosXMM.assign(4,0);
	m_vdWheelPosYMM.assign(4,0);
	m_vdWheelDistMM.assign(4,0);
	m_vdWheelAngRad.assign(4,0);

	m_vdExWheelPosXMM.assign(4,0);
	m_vdExWheelPosYMM.assign(4,0);
	m_vdExWheelDistMM.assign(4,0);
	m_vdExWheelAngRad.assign(4,0);

	m_vdWheelNeutralPos.assign(4,0);
	m_vdDriveGearVelRadS.assign(4,0);
	m_vdSteerGearVelRadS.assign(4,0);
	m_vdDriveGearDltAngRad.assign(4,0);
	m_vdSteerGearAngRad.assign(4,0);
	m_vdDriveGearVelCmdRadS.assign(4,0);
	m_vdSteerGearVelCmdRadS.assign(4,0);
	m_vdSteerGearAngCmdRad.assign(4,0);
	m_vdSteerGearAngTarget1Rad.assign(4,0);
	m_vdDriveGearVelTarget1RadS.assign(4,0);
	m_vdSteerGearAngTarget2Rad.assign(4,0);
	m_vdDriveGearVelTarget2RadS.assign(4,0);
	m_vdSteerGearAngTargetRad.assign(4,0);
	m_vdDriveGearVelTargetRadS.assign(4,0);
	m_vdCtrlVal.assign( 4, std::vector<double> (2,0.0) );
	m_dVel_x_cmd = 0;
	m_dVel_y_cmd = 0;
	m_dVel_rad_cmd = 0;	
	m_dVel_x = 0;
	m_dVel_y = 0;
	m_dVel_rad = 0;	

	// Init params of impedence ctrl
	m_dSpring = 10.0;
	m_dDamp = 2.5;
	m_dVirtualMass = 0.1;
	m_dDPhiMax = 12.0;
	m_dDDPhiMax = 40.0;
// }
}

NeoKinematics::~NeoKinematics(){}

void NeoKinematics::InitParams(int iNumOfJoints, int iWheelDistMM, int iWheelRadiusMM,double dTransMaxVelocity, double dRadMaxVelocity, int iSteerAxisDistToDriveWheelMM, 
			double dCmdRateSec, double dSpring, double dDamp, double dVirtualMass, double dDPhiMax, double dDDPhiMax, double dMaxDriveRadS, double dMaxSteerRadS,
			std::vector<double> vdSteerPosWheelXMM, std::vector<double> vdSteerPosWheelYMM, std::vector<double> vdWheelNeutralPos)
{
	m_iNumOfJoints = iNumOfJoints; 

	//Variables for kinematic parameters
	m_dTransMaxVelocity = dTransMaxVelocity; 
	m_dRadMaxVelocity = dRadMaxVelocity;

	// Geometrical Parameters
	m_iWheelDistMM = iWheelDistMM;
	m_iWheelRadiusMM = iWheelRadiusMM;
	m_iSteerAxisDistToDriveWheelMM = iSteerAxisDistToDriveWheelMM;
	m_dCmdRateSec = dCmdRateSec;
	m_dSpring = dSpring;
	m_dDamp = dDamp;
	m_dVirtualMass = dVirtualMass;
	m_dDPhiMax = dDPhiMax;
	m_dDDPhiMax = dDDPhiMax;
	m_dMaxDriveRadS = dMaxDriveRadS;
	m_dMaxSteerRadS = dMaxSteerRadS;

	// Position of the wheels steering axis in cartesian and polar co-ordinates relative to the robot co-ordinate system
	m_vdWheelPosXMM = vdSteerPosWheelXMM;
	m_vdWheelPosYMM = vdSteerPosWheelYMM;

	// Neutral wheel Position 
	m_vdWheelNeutralPos = vdWheelNeutralPos;

	for(int i = 0; i<4; i++)
	{	
		// Converting degress to radians
		m_vdWheelNeutralPos[i] = NeoMath::DegtoRad(m_vdWheelNeutralPos[i]);
		// provisorial --> skip interpolation
		m_vdSteerGearAngCmdRad[i] = m_vdWheelNeutralPos[i];
		//m_vdAngGearSteerIntpRad[i] = m_UnderCarriagePrms.WheelNeutralPos[i];
		
		// also Init choosen Target angle
		m_vdSteerGearAngTargetRad[i] = m_vdWheelNeutralPos[i];

		//Set inital rotation for each wheel 
		m_vdSteerGearAngRad[i] = m_vdWheelNeutralPos[i];
	}
	

	// calculate polar coords of Wheel Axis in robot coordinate frame
	for(int i=0; i<4; i++)
	{
		m_vdWheelDistMM[i] = sqrt( (m_vdWheelPosXMM[i] * m_vdWheelPosXMM[i]) + (m_vdWheelPosYMM[i] * m_vdWheelPosYMM[i]) );
		m_vdWheelAngRad[i] = NeoMath::atan4quad(m_vdWheelPosYMM[i], m_vdWheelPosXMM[i]);
	}
	WheelPositionCalc();


}

// void NeoKinematics::InitialiseWheelPosition()
// {
// 	for(int i = 0; i<4; i++)
// 	{	
// 		// Converting degress to radians
// 		m_vdWheelNeutralPos[i] = NeoMath::DegtoRad(m_vdWheelNeutralPos[i]);
// 		// provisorial --> skip interpolation
// 		m_vdSteerGearAngCmdRad[i] = m_vdWheelNeutralPos[i];
// 		//m_vdAngGearSteerIntpRad[i] = m_UnderCarriagePrms.WheelNeutralPos[i];
		
// 		// also Init choosen Target angle
// 		m_vdSteerGearAngTargetRad[i] = m_vdWheelNeutralPos[i];

// 		//Set inital rotation for each wheel 
// 		m_vdSteerGearAngRad[i] = m_vdWheelNeutralPos[i];
// 	}
	

// 	// calculate polar coords of Wheel Axis in robot coordinate frame
// 	for(int i=0; i<4; i++)
// 	{
// 		m_vdWheelDistMM[i] = sqrt( (m_vdWheelPosXMM[i] * m_vdWheelPosXMM[i]) + (m_vdWheelPosYMM[i] * m_vdWheelPosYMM[i]) );
// 		m_vdWheelAngRad[i] = NeoMath::atan4quad(m_vdWheelPosYMM[i], m_vdWheelPosXMM[i]);
// 	}
// 	WheelPositionCalc();
// }

void NeoKinematics::WheelPositionCalc()
{
	for(int i = 0; i<4; i++)
	{
		// std::cout<<m_vdSteerGearAngRad[i]<<std::endl;
		// calculate current geometry of robot (exact wheel position, taking into account steering offset of wheels)
		m_vdExWheelPosXMM[i] = m_vdWheelPosXMM[i] + m_iSteerAxisDistToDriveWheelMM * sin(m_vdSteerGearAngRad[i]);
		m_vdExWheelPosYMM[i] = m_vdWheelPosYMM[i] - m_iSteerAxisDistToDriveWheelMM * cos(m_vdSteerGearAngRad[i]);
				
		// calculate distance from platform center to wheel center
		m_vdExWheelDistMM[i] = sqrt( (m_vdExWheelPosXMM[i] * m_vdExWheelPosXMM[i]) + (m_vdExWheelPosYMM[i] * m_vdExWheelPosYMM[i]) );
		// std::cout<<m_vdExWheelDistMM[i]<<std::endl;

		// calculate direction of rotational vector
		m_vdExWheelAngRad[i] = NeoMath::atan4quad( m_vdExWheelPosYMM[i], m_vdExWheelPosXMM[i]);
	}

}

void NeoKinematics::InverseKinematicsCalc()
{
	// Temporary variable to store the velocities in mm/s

	double dtempAxVelXRobMMS, dtempAxVelYRobMMS;	

	if((m_dVel_x_cmd == 0) && (m_dVel_y_cmd == 0) && (m_dVel_rad_cmd == 0))
	{
		for(int i = 0; i<4; i++)
		{
			m_vdSteerGearAngTarget1Rad[i] = m_vdSteerGearAngRad[i];
			m_vdDriveGearVelTarget1RadS[i] = 0.0;
			m_vdSteerGearAngTarget2Rad[i] = m_vdSteerGearAngRad[i];
			m_vdDriveGearVelTarget2RadS[i] = 0.0;
		}
		return;
	}

	// calculate sets of possible Steering Angle // Drive-Velocity combinations
	for (int i = 0; i<4; i++)
	{	
		// calculate velocity and direction of single wheel motion
		// Translational Portion
		dtempAxVelXRobMMS = m_dVel_x_cmd;
		dtempAxVelYRobMMS = m_dVel_y_cmd;
		// Rotational Portion
		// std::cout<<"Here"<<-cos(m_vdWheelAngRad[i])<<std::endl;
		dtempAxVelXRobMMS += m_dVel_rad_cmd * m_vdExWheelDistMM[i] * -sin(m_vdExWheelAngRad[i]);
		dtempAxVelYRobMMS += m_dVel_rad_cmd * m_vdExWheelDistMM[i] * cos(m_vdExWheelAngRad[i]);
		
		// calculate resulting steering angle 
		// Wheel has to move in direction of resulting velocity vector of steering axis 
/*		std::cout<<"tempX:"<<dtempAxVelXRobMMS<<std::endl;
		std::cout<<"tempY:"<<dtempAxVelYRobMMS<<std::endl;*/
		m_vdSteerGearAngTarget1Rad[i] = NeoMath::atan4quad(dtempAxVelYRobMMS, dtempAxVelXRobMMS);
		// calculate corresponding angle in opposite direction (+180 degree)
		m_vdSteerGearAngTarget2Rad[i] = m_vdSteerGearAngTarget1Rad[i] + NeoMath::PI;
		NeoMath::PiNormalization(m_vdSteerGearAngTarget2Rad[i]);
		
		// calculate absolute value of rotational rate of driving wheels in rad/s
		m_vdDriveGearVelTarget1RadS[i] = sqrt( (dtempAxVelXRobMMS * dtempAxVelXRobMMS) + 
						   (dtempAxVelYRobMMS * dtempAxVelYRobMMS) ) / (double)m_iWheelRadiusMM;
		// now adapt to direction (forward/backward) of wheel
		m_vdDriveGearVelTarget2RadS[i] = - m_vdDriveGearVelTarget1RadS[i];
/*		std::cout<<"Drive:"<<m_vdDriveGearVelTarget1RadS[i]<<std::endl;
		std::cout<<"Drive:"<<m_vdDriveGearVelTarget2RadS[i]<<std::endl;
		std::cout<<"steer"<<m_vdSteerGearAngTarget1Rad[i]<<std::endl;
		std::cout<<"steer"<<m_vdSteerGearAngTarget2Rad[i]<<std::endl;*/
	}
}

void NeoKinematics::ForwardKinematicsCalc()
{
			// declare auxilliary variables
	double dtempVelXRobMMS;		// Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
	double dtempVelYRobMMS;		// Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)
	double dtempRotRobRADPS;	// Robot-Rotation-Rate in rad/s (in Robot-Coordinateframe)
	double dtempDiffXMM;		// Difference in X-Coordinate of two wheels in mm
	double dtempDiffYMM;		// Difference in Y-Coordinate of two wheels in mm
	double dtempRelPhiWheelsRAD;	// Angle between axis of two wheels w.r.t the X-Axis of the Robot-Coordinate-System in rad
	double dtempRelDistWheelsMM;	// distance of two wheels in mm 
	double dtempRelPhiWheel1RAD;	// Steering Angle of (im math. pos. direction) first Wheel w.r.t. the linking axis of the two wheels
	double dtempRelPhiWheel2RAD;	// Steering Angle of (im math. pos. direction) first Wheel w.r.t. the linking axis of the two wheels
	std::vector<double> vdtempVelWheelMMS(4);	// Wheel-Velocities (all Wheels) in mm/s
	// vdtempVelWheelMMS.assign(1,0);
	// initial values
	dtempVelXRobMMS = 0;			// Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
	dtempVelYRobMMS = 0;			// Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)
	dtempRotRobRADPS = 0;
	// calculate corrected wheel velocities
	// std::cout<<"Desired vel:";
	for(int i = 0; i<4; i++)
	{
		// calc effective Driving-Velocity
		vdtempVelWheelMMS[i] = m_iWheelRadiusMM * (m_vdDriveGearVelRadS[i]);
		// std::cout<<"Desired vel:"<<m_vdDriveGearVelRadS[i]<<std::endl;
	}

	// calculate rotational rate of robot and current "virtual" axis between all wheels
	for(int i = 0; i< (4-1) ; i++)
	{
		// calc Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
		dtempDiffXMM = m_vdExWheelPosXMM[i+1] - m_vdExWheelPosXMM[i];
		dtempDiffYMM = m_vdExWheelPosYMM[i+1] - m_vdExWheelPosYMM[i];
		dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
		dtempRelPhiWheelsRAD = NeoMath::atan4quad( dtempDiffYMM, dtempDiffXMM );

		// transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
		dtempRelPhiWheel1RAD = m_vdSteerGearAngRad[i] - dtempRelPhiWheelsRAD;
		dtempRelPhiWheel2RAD = m_vdSteerGearAngRad[i+1] - dtempRelPhiWheelsRAD;

		dtempRotRobRADPS += (vdtempVelWheelMMS[i+1] * sin(dtempRelPhiWheel2RAD) - vdtempVelWheelMMS[i] * sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;
	}

	// calculate last missing axis (between last wheel and 1.)
	// calc. Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
	dtempDiffXMM = m_vdExWheelPosXMM[0] - m_vdExWheelPosXMM[4-1];
	dtempDiffYMM = m_vdExWheelPosYMM[0] - m_vdExWheelPosYMM[4-1];
	dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
	dtempRelPhiWheelsRAD = NeoMath::atan4quad( dtempDiffYMM, dtempDiffXMM );

	// transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
	dtempRelPhiWheel1RAD = m_vdSteerGearAngRad[4-1] - dtempRelPhiWheelsRAD;
	dtempRelPhiWheel2RAD = m_vdSteerGearAngRad[0] - dtempRelPhiWheelsRAD;

	// close calculation of robots rotational velocity
	dtempRotRobRADPS += (vdtempVelWheelMMS[0]*sin(dtempRelPhiWheel2RAD) - vdtempVelWheelMMS[4-1]*sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;

	// calculate linear velocity of robot
	for(int i = 0; i<4; i++)
	{
		
		dtempVelXRobMMS += vdtempVelWheelMMS[i]*cos(m_vdSteerGearAngRad[i]);
		dtempVelYRobMMS += vdtempVelWheelMMS[i]*sin(m_vdSteerGearAngRad[i]);
		// std::cout<<"dtempVelXRobMMS:"<<vdtempVelWheelMMS[i]<<std::endl;
		// std::cout<<"dtempVelYRobMMS:"<<m_vdSteerGearAngRad[i]<<std::endl;
		// std::cout<<"dtempVelZRobMMS:"<<dtempVelXRobMMS<<std::endl;
			// std::cout<<dtempVelXRobMMS<<std::endl;


	}

	// assign rotational velocities for output
	// m_dRotRobRadS = dtempRotRobRADPS/m_iNumOfJoints;
	m_dVel_rad = dtempRotRobRADPS/4;
	// std::cout<<dtempVelXRobMMS<<std::endl;
	// assign linear velocity of robot for output
	m_dVel_x = dtempVelXRobMMS/4;
	m_dVel_y = dtempVelYRobMMS/4;
}


// Public functions
void NeoKinematics::SetRequiredWheelPoses(std::vector<double> vdDriveGearVelRadS,std::vector<double> vdSteerGearVelRadS,std::vector<double> vdDriveGearDltAngRad,std::vector<double> vdSteerGearAngRad)
{

	// std::cout<<"h1"<<vdDriveGearVelRadS[0]<<std::endl;
	m_vdDriveGearVelRadS = vdDriveGearVelRadS;
	m_vdSteerGearVelRadS = vdSteerGearVelRadS;
	m_vdDriveGearDltAngRad = vdDriveGearDltAngRad;
	m_vdSteerGearAngRad = vdSteerGearAngRad;

	WheelPositionCalc();

	ForwardKinematicsCalc();



}

void NeoKinematics::SetRequiredVelocity(double dVel_x_cmd,  double dVel_y_cmd, double dVel_rad_cmd)
{
	// declare auxiliary variables
	double dCurrentPosWheelRAD;
	double dtempDeltaPhi1RAD, dtempDeltaPhi2RAD;	// difference between possible steering angels and current steering angle
	double dtempDeltaPhiCmd1RAD, dtempDeltaPhiCmd2RAD;	// difference between possible steering angels and last target steering angle
	double dtempWeightedDelta1RAD, dtempWeightedDelta2RAD; // weighted Summ of the two distance values

	// copy function parameters to member variables
	m_dVel_x_cmd = dVel_x_cmd;
	m_dVel_y_cmd = dVel_y_cmd;
	m_dVel_rad_cmd = dVel_rad_cmd;

	InverseKinematicsCalc();
	// Determining the optimal platform configuration
		// determine optimal Pltf-Configuration
	for (int i = 0; i<4; i++)
	{
		// Normalize Actual Wheel Position before calculation
		dCurrentPosWheelRAD = m_vdSteerGearAngRad[i];
		NeoMath::PiNormalization(dCurrentPosWheelRAD);
		
		// Calculate differences between current config to possible set-points
		dtempDeltaPhi1RAD = m_vdSteerGearAngTarget1Rad[i] - dCurrentPosWheelRAD;
		dtempDeltaPhi2RAD = m_vdSteerGearAngTarget2Rad[i] - dCurrentPosWheelRAD;
		NeoMath::PiNormalization(dtempDeltaPhi1RAD);
		NeoMath::PiNormalization(dtempDeltaPhi2RAD);
		// Calculate differences between last steering target to possible set-points
		dtempDeltaPhiCmd1RAD = m_vdSteerGearAngTarget1Rad[i] - m_vdSteerGearAngTargetRad[i];
		dtempDeltaPhiCmd2RAD = m_vdSteerGearAngTarget2Rad[i] - m_vdSteerGearAngTargetRad[i];
		NeoMath::PiNormalization(dtempDeltaPhiCmd1RAD);
		NeoMath::PiNormalization(dtempDeltaPhiCmd2RAD);
		
		// determine optimal setpoint value
		// 1st which set point is closest to current cinfog
		//     but: avoid permanent switching (if next target is about PI/2 from current config)
		// 2nd which set point is closest to last set point
		// "fitness criteria" to choose optimal set point:
		// calculate accumulted (+ weighted) difference between targets, current config. and last command
		dtempWeightedDelta1RAD = 0.6*fabs(dtempDeltaPhi1RAD) + 0.4*fabs(dtempDeltaPhiCmd1RAD);
		dtempWeightedDelta2RAD = 0.6*fabs(dtempDeltaPhi2RAD) + 0.4*fabs(dtempDeltaPhiCmd2RAD);

		// check which set point "minimizes fitness criteria"
		if (dtempWeightedDelta1RAD <= dtempWeightedDelta2RAD)
		{
			// Target1 is "optimal"
			m_vdDriveGearVelTargetRadS[i] = m_vdDriveGearVelTarget1RadS[i];
			m_vdSteerGearAngTargetRad[i] = m_vdSteerGearAngTarget1Rad[i];
		}
		else
		{
			// Target2 is "optimal"
			m_vdDriveGearVelTargetRadS[i] = m_vdDriveGearVelTarget2RadS[i];
			m_vdSteerGearAngTargetRad[i] = m_vdSteerGearAngTarget2Rad[i];
		}


	}

}
void NeoKinematics::CtrlStepCalc()
{
	double dCurrentPosWheelRAD;
	double dDeltaPhi;
	double dForceDamp, dForceProp, dAccCmd, dVelCmdInt; // PI- and Impedance-Ctrl

	if ((m_dVel_x_cmd == 0) && (m_dVel_y_cmd == 0) && (m_dVel_rad_cmd == 0))
	{
		m_vdDriveGearVelCmdRadS.assign(4,0.0);		// set velocity for drives to zero
		m_vdSteerGearVelCmdRadS.assign(4,0.0);		// set velocity for steers to zero

		// set internal states of controller to zero
		for(int i=0; i<4; i++)
		{
			m_vdCtrlVal[i][0] = 0.0;
			m_vdCtrlVal[i][1] = 0.0;
		}
		return;
	}
	for (int i=0; i<4; i++)
	{
		// provisorial --> skip interpolation and always take Target
		m_vdDriveGearVelCmdRadS[i] = m_vdDriveGearVelTargetRadS[i];
		m_vdSteerGearAngCmdRad[i] = m_vdSteerGearAngTargetRad[i];

	}

	for (int i = 0; i<4; i++)
	{

		// Normalize Actual Wheel Position before calculation
		dCurrentPosWheelRAD = m_vdSteerGearAngRad[i];
		NeoMath::PiNormalization(dCurrentPosWheelRAD);
		// if(dCurrentPosWheelRAD != 0)
		// {std::cout<<"Debug::check::MAX:"<<dCurrentPosWheelRAD<<std::endl;}
		dDeltaPhi = m_vdSteerGearAngCmdRad[i] - dCurrentPosWheelRAD;
		NeoMath::PiNormalization(dDeltaPhi);

		// std::cout<<dDeltaPhi<<std::endl;

		// Impedance-Ctrl
		// Calculate resulting desired forces, velocities
		// double dForceDamp, dForceProp, dAccCmd, dVelCmdInt;
		dForceDamp = - m_dDamp * m_vdCtrlVal[i][1];
		dForceProp = m_dSpring * dDeltaPhi;
		dAccCmd = (dForceDamp + dForceProp) / m_dVirtualMass;
		if (dAccCmd > m_dDDPhiMax)
		{
			dAccCmd = m_dDDPhiMax;
		}
		else if (dAccCmd < -m_dDDPhiMax)
		{
			dAccCmd = -m_dDDPhiMax;
		}
		dVelCmdInt = m_vdCtrlVal[i][1] + m_dCmdRateSec * dAccCmd;
		if (dVelCmdInt > m_dDPhiMax)
		{
			dVelCmdInt = m_dDPhiMax;
		}
		else if (dVelCmdInt < -m_dDPhiMax)
		{
			dVelCmdInt = -m_dDPhiMax;
		}
		// Store internal ctrlr-states
		m_vdCtrlVal[i][0] = dDeltaPhi;
		m_vdCtrlVal[i][1] = dVelCmdInt;
		// set outputs
		// std::cout<<dVelCmdInt<<std::endl;

		m_vdSteerGearVelCmdRadS[i] = dVelCmdInt;

		// Checking for steering velocity rates
		if(fabs(m_vdSteerGearVelCmdRadS[i]) > m_dRadMaxVelocity)
		{
			if (m_vdSteerGearVelCmdRadS[i] > 0)

				m_vdSteerGearVelCmdRadS[i] = m_dRadMaxVelocity;
			else
				m_vdSteerGearVelCmdRadS[i] = -m_dRadMaxVelocity;
		}
	}
}
void NeoKinematics:: GetPltfVel(double *dDeltaVel_x,  double *dDeltaVel_y, double *dDeltaVel_rad,double *dVel_x,  double *dVel_y, double *dVel_rad)
{
	*dVel_x = m_dVel_x;
	*dVel_y = m_dVel_y;
	// dRotRobRadS = m_dRotRobRadS;
	*dVel_rad = m_dVel_rad;
	// std::cout<<"cmd_vel:x"<<dVel_x_cmd<<std::endl;

	// calculate travelled distance and angle (from velocity) for output
	// ToDo: make sure this corresponds to cycle-freqnecy of calling node
	//       --> specify via config file
	*dDeltaVel_x = m_dVel_x * m_dCmdRateSec;
	*dDeltaVel_y = m_dVel_y * m_dCmdRateSec;
	*dDeltaVel_rad = m_dVel_rad * m_dCmdRateSec;
}

void NeoKinematics::GetRefreshedCtrlState(	std::vector<double> *vdDriveGearVelRadS, std::vector<double> *vdSteerGearVelRadS, std::vector<double> *vdSteerGearAngRad, double *dVel_x, double *dVel_y, double *dVel_rad)
{
	// if condition ? 
	CtrlStepCalc();
	*vdDriveGearVelRadS = m_vdDriveGearVelCmdRadS;
	*vdSteerGearVelRadS = m_vdSteerGearVelCmdRadS;
	*vdSteerGearAngRad = m_vdSteerGearAngCmdRad;
	*dVel_x = m_dVel_x_cmd;
	*dVel_y = m_dVel_y_cmd;
	*dVel_rad = m_dVel_rad_cmd;

}

// void NeoKinematics:: operator=(NeoKinematics const & GeomCtrl)
// {
// 	// Actual Values for PltfMovement (calculated from Actual Wheelspeeds)
// 	m_dVel_x_cmd = GeomCtrl.m_dVel_x_cmd;
// 	m_dVel_y_cmd = GeomCtrl.m_dVel_y_cmd;
// 	m_dVel_rad_cmd = GeomCtrl.m_dVel_rad_cmd;

// 	m_dVel_x = GeomCtrl.m_dVel_x;
// 	m_dVel_y = GeomCtrl.m_dVel_y;
// 	m_dVel_rad = GeomCtrl.m_dVel_rad;
// 	m_vdWheelPosXMM = GeomCtrl.m_vdWheelPosXMM;
// 	m_vdWheelPosYMM = GeomCtrl.m_vdWheelPosYMM;
// 	m_vdWheelDistMM = GeomCtrl.m_vdWheelDistMM;
// 	m_vdWheelAngRad = GeomCtrl.m_vdWheelAngRad;
// 	m_vdExWheelPosXMM = GeomCtrl.m_vdExWheelPosXMM;
// 	m_vdExWheelPosYMM = GeomCtrl.m_vdExWheelPosYMM;
// 	m_vdExWheelDistMM = GeomCtrl.m_vdExWheelDistMM;
// 	m_vdExWheelAngRad = GeomCtrl.m_vdExWheelAngRad;
// 	m_vdDriveGearVelRadS = GeomCtrl.m_vdDriveGearVelRadS;
// 	m_vdSteerGearVelRadS = GeomCtrl.m_vdSteerGearVelRadS;
// 	m_vdDriveGearDltAngRad = GeomCtrl.m_vdDriveGearDltAngRad;
// 	m_vdSteerGearAngRad = GeomCtrl.m_vdSteerGearAngRad;
// 	m_vdDriveGearVelCmdRadS = GeomCtrl.m_vdDriveGearVelCmdRadS;
// 	m_vdSteerGearVelCmdRadS = GeomCtrl.m_vdSteerGearVelCmdRadS;
// 	m_vdSteerGearAngCmdRad = GeomCtrl.m_vdSteerGearAngCmdRad;
// 	m_vdSteerGearAngTarget1Rad = GeomCtrl.m_vdSteerGearAngTarget1Rad;
// 	m_vdDriveGearVelTarget1RadS = GeomCtrl.m_vdDriveGearVelTarget1RadS;
// 	m_vdSteerGearAngTarget2Rad = GeomCtrl.m_vdSteerGearAngTarget2Rad;
// 	m_vdDriveGearVelTarget2RadS = GeomCtrl.m_vdDriveGearVelTarget2RadS;
// 	m_iNumOfJoints = GeomCtrl.m_iNumOfJoints;
// 	m_dTransMaxVelocity = GeomCtrl.m_dTransMaxVelocity;
// 	m_dRadMaxVelocity = GeomCtrl.m_dRadMaxVelocity;
// 	m_iWheelDistMM = GeomCtrl.m_iWheelDistMM;
// 	m_iWheelRadiusMM = GeomCtrl.m_iWheelRadiusMM;
// 	m_iSteerAxisDistToDriveWheelMM = GeomCtrl.m_iSteerAxisDistToDriveWheelMM;
// 	m_dCmdRateSec = GeomCtrl.m_dCmdRateSec;
// 	m_dSpring = GeomCtrl.m_dSpring;
// 	m_dDamp = GeomCtrl.m_dDamp;
// 	m_dVirtualMass = GeomCtrl.m_dVirtualMass;
// 	m_dDPhiMax = GeomCtrl.m_dDPhiMax;
// 	m_dDDPhiMax = GeomCtrl.m_dDDPhiMax;
// 	m_dMaxDriveRadS = GeomCtrl.m_dMaxDriveRadS;
// 	m_dMaxSteerRadS = GeomCtrl.m_dMaxSteerRadS;
// 	m_vdWheelNeutralPos = GeomCtrl.m_vdWheelNeutralPos;
// 	m_vdCtrlVal = GeomCtrl.m_vdCtrlVal;



// }