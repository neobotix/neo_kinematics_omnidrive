#include <neo_kinematics_omnidrive/NeoKinematics.h>

// void NeoKinematics::WheelPositionCalc()
// {
// 	for(int i = 0; i<4; i++)
// 	{
// 		// calculate current geometry of robot (exact wheel position, taking into account steering offset of wheels)
// 		m_vdExWheelXPosMM[i] = m_vdWheelXPosMM[i] + m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM * sin(m_vdAngGearSteerRad[i]);
// 		m_vdExWheelYPosMM[i] = m_vdWheelYPosMM[i] - m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM * cos(m_vdAngGearSteerRad[i]);
				
// 		// calculate distance from platform center to wheel center
// 		m_vdExWheelDistMM[i] = sqrt( (m_vdExWheelXPosMM[i] * m_vdExWheelXPosMM[i]) + (m_vdExWheelYPosMM[i] * m_vdExWheelYPosMM[i]) );
		
// 		// calculate direction of rotational vector
// 		m_vdExWheelAngRad[i] = MathSup::atan4quad( m_vdExWheelYPosMM[i], m_vdExWheelXPosMM[i]);
// 	}

// }

void NeoKinematics::InitParams(int iNumOfJoints, int iWheelDistMM, int iWheelRadiusMM,double dTransMaxVelocity, double dRadMaxVelocity, int iSteerAxisDistToDriveWheelMM, 
			double dCmdRateSec, double dSpring, double dDamp, double dVirtualMass, double dDPhiMax, double dDDPhiMax, double dMaxDriveRadS, double dMaxSteerRadS,
			std::vector<double> vdSteerPosWheelXMM, std::vector<double> vdSteerPosWheelYMM, std::vector<double> vdSteerWheelDistMM, std::vector<double> vdSteerWheelAngRad,
			std::vector<double> vdPosWheelXMM, std::vector<double> vdPosWheelYMM, std::vector<double> vdWheelDistMM, std::vector<double> vdWheelAngRad,
			std::vector<double> vdWheelNeutralPos)
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
m_vdSteerPosWheelXMM = vdSteerPosWheelXMM;
m_vdSteerPosWheelYMM = vdSteerPosWheelYMM;
m_vdSteerWheelDistMM = vdSteerWheelDistMM;
m_vdSteerWheelAngRad = vdSteerWheelAngRad;

// Position of the wheels in cartesian and polar co-ordinates relative to the robot co-ordinate system
m_vdPosWheelXMM = vdPosWheelXMM;
m_vdPosWheelYMM = vdPosWheelYMM;
m_vdWheelDistMM = vdWheelDistMM;
m_vdWheelAngRad = vdWheelAngRad;

// Neutral wheel Position 
m_vdWheelNeutralPos = vdWheelNeutralPos;

}
