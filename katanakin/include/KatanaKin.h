/*
 * KatanaKin.h
 *
 *  Created on: Jan 22, 2013
 *      Author: seungsu
 */

#ifndef KATANAKIN_H_
#define KATANAKIN_H_

#include "MathLib.h"
#include "IKGroupSolver.h"
#include "sKinematics.h"

#define ROBOT_DOF 6
#define IK_CONSTRAINTS 6
#define IK_DIRAIXS 2
#define DT (1./4.)

enum KATANA_MODEL
{
	KATANA_T,
	KATANA_G
};

class KatanaKin{
protected:
	MathLib::Vector        mRotDirection;

	MathLib::IKGroupSolver mIKSolver;
	MathLib::Matrix        mJacobianPos;
	MathLib::Matrix        mJacobianDir;
	MathLib::Matrix        mJacobian;
	MathLib::Matrix        mJacobianDirY, mJacobianDirZ;
	MathLib::Matrix        mJacobian9;

	MathLib::Vector        mJointPos;
	MathLib::Vector        mJointWeights;
	MathLib::Vector        mJointVelLimitsDn, mJointVelLimitsUp;

	KATANA_MODEL mKatanaModel;
	int mDOF;

	void initiKinematics(void);
	void SolveIK(int linkindex, double* pPos, double* pDirY, double* pDirZ, double dirWeight, double* pAngles, double& posError, double& dirError);
	void SolveIK(int linkindex, double* pPos, int axis, double* pDir, double dirWeight, double* pAngles, double& posError, double& dirError);
public:
	sKinematics           *mKinematicChain;

	KatanaKin(KATANA_MODEL pModel);
	~KatanaKin();

	void SetJoints(double* pAngles);
	void GetEndPos(double* pPos);
	void GetEndDir(int pIndex, double* pDir);
	void GetJoints(double* pAngles);

	void SolveIKWrist(double* pPos, int axis, double* pDir, double dirWeight, double* pAngles, double& posError, double& dirError);
	void SolveIKWrist(double* pPos, double* pDirY, double* pDirZ, double dirWeight, double* pAngles, double& posError, double& dirError);
	void SolveIKGripper(double* pPos, int axis, double* pDir, double dirWeight, double* pAngles, double& posError, double& dirError);
};

#endif /* KATANAKIN_H_ */
