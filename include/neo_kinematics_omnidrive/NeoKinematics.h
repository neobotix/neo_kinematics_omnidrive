#ifndef NEOKINEMATICS_INCLUDEDEF_H
#define NEOKINEMATICS_INCLUDEDEF_H
#include <vector>
#include<iostream>
#include<NeoMath.h>
#include <math.h>
class NeoKinematics
{

public:	
	// Number of joints
	int m_iNumOfJoints; 

	//Variables for kinematic parameters
	double m_dTransMaxVelocity, m_dRadMaxVelocity;

	//Variables Geometrical Parameters
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
	std::vector<double> m_vdWheelPosXMM;
	std::vector<double> m_vdWheelPosYMM;
	std::vector<double> m_vdWheelDistMM;
	std::vector<double> m_vdWheelAngRad;
	std::vector<double> m_vdWheelNeutralPos;

		std::vector<double> m_vdExWheelPosXMM;
	std::vector<double> m_vdExWheelPosYMM;
	std::vector<double> m_vdExWheelDistMM;
	std::vector<double> m_vdExWheelAngRad;

	// Variables to denote(calculated) platform movement
	double m_dVel_x;
	double m_dVel_y;
	double m_dVel_rad; 

	// Wheel speed measurement read from the motor controllers
	std::vector<double> m_vdDriveGearVelRadS;
	std::vector<double> m_vdSteerGearVelRadS;
	std::vector<double> m_vdDriveGearDltAngRad;
	std::vector<double> m_vdSteerGearAngRad;

	// Variable for desired platform movement
	double m_dVel_x_cmd ;
	double m_dVel_y_cmd ;
	double m_dVel_rad_cmd ;		

	// Vectors denoting the commanded wheel speeds set to the ELMO Motor Controller
	std::vector<double> m_vdDriveGearVelCmdRadS;
	std::vector<double> m_vdSteerGearVelCmdRadS;
	std::vector<double> m_vdSteerGearAngCmdRad;

	// Neutral wheel Position 
	std::vector<double> vdWheelNeutralPos;
	std::vector< std::vector<double> > m_vdCtrlVal;
	// Target Wheelspeed and -angle (calculated from desired Pltf-Movement with Inverse without controle!)
	// This Values might not be valid (to high step response in steering rate, ...) for commanding the drives
	std::vector<double> m_vdSteerGearAngTarget1Rad; // alternativ 1 for steering angle
	std::vector<double> m_vdDriveGearVelTarget1RadS;
	std::vector<double> m_vdSteerGearAngTarget2Rad; // alternativ 2 for steering angle (+/- PI)
	std::vector<double> m_vdDriveGearVelTarget2RadS;
	std::vector<double> m_vdSteerGearAngTargetRad; // choosen alternativ for steering angle
	std::vector<double> m_vdDriveGearVelTargetRadS;

	void InitParams(int iNumOfJoints, int iWheelDistMM, int iWheelRadiusMM,double dTransMaxVelocity, double dRadMaxVelocity, int iSteerAxisDistToDriveWheelMM, 
					double dCmdRateSec, double dSpring, double dDamp, double dVirtualMass, double dDPhiMax, double dDDPhiMax, double dMaxDriveRadS, double dMaxSteerRadS,
					std::vector<double> vdSteerPosWheelXMM, std::vector<double> vdSteerPosWheelYMM,
					std::vector<double> vdWheelNeutralPos);
	// void InitialiseWheelPosition();
	void SetRequiredVelocity(double dVel_x_cmd,  double dVel_y_cmd, double dVel_rad_cmd);
	void SetRequiredWheelPoses(std::vector<double> vdDriveGearVelRadS,std::vector<double> vdSteerGearVelRadS,std::vector<double> vdDriveGearDltAngRad,std::vector<double> vdSteerGearAngRad);
	void GetPltfVel(double *dDeltaVel_x,  double *dDeltaVel_y, double *dDeltaVel_rad,double *dVel_x,  double *dVel_y, double *dVel_rad);
	void GetRefreshedCtrlState(	std::vector<double> *vdDriveGearVelRadS, std::vector<double> *vdSteerGearVelRadS, std::vector<double> *vdSteerGearAngRad, double *dVel_x, double *dVel_y, double *dVel_rad);


NeoKinematics();
~NeoKinematics();
	// void operator=();

void operator=(NeoKinematics const & GeomCtrl);

private:
	void InverseKinematicsCalc();
	void ForwardKinematicsCalc();
	void CtrlStepCalc();
	void WheelPositionCalc();





};


#endif
