#ifndef NEOKINEMATICS_INCLUDEDEF_H
#define NEOKINEMATICS_INCLUDEDEF_H
#include <vector>
#include<iostream>
class NeoKinematics
{

public:
	int m_iNumOfJoints; 

	//Variables for kinematic parameters
	double m_dTransMaxVelocity, m_dRadMaxVelocity;

	// Geometrical Parameters
	int m_iWheelDistMM;
	int m_iWheelRadiusMM;
	int m_iSteerAxisDistToDriveWheelMM;
	double m_dCmdRateSec;
	double m_dSpring;
	double m_dDamp;
	double m_dVirtualMass;
	double m_dDPhiMax;
	double m_dDDPhiMax;
	double m_dMaxDriveRadS;
	double m_dMaxSteerRadS;

	// Position of the wheels steering axis in cartesian and polar co-ordinates relative to the robot co-ordinate system
	std::vector<double> m_vdSteerPosWheelXMM;
	std::vector<double> m_vdSteerPosWheelYMM;
	std::vector<double> m_vdSteerWheelDistMM;
	std::vector<double> m_vdSteerWheelAngRad;

	// Position of the wheels in cartesian and polar co-ordinates relative to the robot co-ordinate system
	std::vector<double> m_vdPosWheelXMM;
	std::vector<double> m_vdPosWheelYMM;
	std::vector<double> m_vdWheelDistMM;
	std::vector<double> m_vdWheelAngRad;
	std::vector<double> m_vdWheelNeutralPos;

	// Neutral wheel Position 
	std::vector<double> vdWheelNeutralPos;
	void InitParams(int iNumOfJoints, int iWheelDistMM, int iWheelRadiusMM,double dTransMaxVelocity, double dRadMaxVelocity, int iSteerAxisDistToDriveWheelMM, 
					double dCmdRateSec, double dSpring, double dDamp, double dVirtualMass, double dDPhiMax, double dDDPhiMax, double dMaxDriveRadS, double dMaxSteerRadS,
					std::vector<double> vdSteerPosWheelXMM, std::vector<double> vdSteerPosWheelYMM, std::vector<double> vdSteerWheelDistMM, std::vector<double> vdSteerWheelAngRad,
					std::vector<double> vdPosWheelXMM, std::vector<double> vdPosWheelYMM, std::vector<double> vdWheelDistMM, std::vector<double> vdWheelAngRad,
					std::vector<double> vdWheelNeutralPos);
	// void IK_VelCalc();
	// void FK_VelCalc();
	// void GetPltfVel();
	// void GetRefreshedCtrlState();

private:

	// void WheelPositionCalc(std::vector<double> *vdDriveVelGearRadS, std::vector<double> *vdSteerVelGearRadS, 
	// 	std::vector<double> *vdDriveDeltaAngGearRad, std::vector<double> *vdSteerDeltaAngGearRad);


NeoKinematics(){}
~NeoKinematics(){}

};


#endif
