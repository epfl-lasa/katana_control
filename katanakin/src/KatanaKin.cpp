/*
 * KatanaKin.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: seungsu
 */

#include "KatanaKin.h"

void KatanaKin::initiKinematics(){
	mKinematicChain = new sKinematics(6, DT);

	if( mKatanaModel == KATANA_T )
	{
		// a d alpha theta
		mKinematicChain->setDH(0,  0.0000,  0.0000     ,  M_PI_2,     0.0, 1,  -3.000000, 3.000000, 100.0);
		mKinematicChain->setDH(1,  0.1900,  0.0000     ,     0.0,     0.0, 1,  -0.210000, 2.100000, 100.0);
		mKinematicChain->setDH(2,  0.1390,  0.0000     , -M_PI  ,     0.0, 1,  -2.190000, 2.100000, 100.0);
		mKinematicChain->setDH(3,  0.0000,  0.0000     ,  M_PI_2, -M_PI_2, 1,  -2.000000, 1.980000, 100.0);
		mKinematicChain->setDH(4,  0.0000, -0.1473+0.02, -M_PI_2,  M_PI_2, 1,  -2.900000, 3.000000, 100.0);
		mKinematicChain->setDH(5,  0.0000,  0.1900     ,     0.0,     60.0*M_PI/180., 1,  -2.150000, 3.700000, 100.0);
		//mKinematicChain->setDH(5,  0.0000, -0.0375,     0.0,     0.0, 1,  -2.150000, 3.700000, 100.0);

		mDOF = 6;

	}
	else if( mKatanaModel == KATANA_G )
	{
		mKinematicChain->setDH(0,  0.0000,  0.0000     ,  M_PI_2,     0.0, 1,  -3.000000, 3.000000, 100.0);
		mKinematicChain->setDH(1,  0.1900,  0.0000     ,     0.0,     0.0, 1,  -0.210000, 2.100000, 100.0);
		mKinematicChain->setDH(2,  0.1390,  0.0000     , -M_PI  ,     0.0, 1,  -2.190000, 2.100000, 100.0);
		mKinematicChain->setDH(3,  0.0000,  0.0000     ,  M_PI_2, -M_PI_2, 1,  -2.000000, 1.980000, 100.0);
		mKinematicChain->setDH(4,  0.0000, -0.1473+0.02, -M_PI_2,  M_PI_2, 1,  -2.900000, 3.000000, 100.0);
		mKinematicChain->setDH(5,  0.0000,  0.1700     ,     0.0,     0.0, 0,  -2.000000, 0.600000, 100.0);

//		// a d alpha theta
//		mKinematicChain->setDH(0,  0.000,  0.0000,     0.0,    0.0, 1,  -3.025529, 3.013311, 100.0);
//		mKinematicChain->setDH(1,  0.000,  0.0000,  M_PI_2,    0.0, 1,  -0.274889, 2.168572, 100.0);
//		mKinematicChain->setDH(2,  0.190,  0.0000,     0.0,    0.0, 1,  -2.221804, 2.141519, 100.0);
//		mKinematicChain->setDH(3,  0.139,  0.0000,     0.0,    0.0, 1,  -3.551745, 0.462512, 100.0);
//		mKinematicChain->setDH(4,  0.000,  0.1473, -M_PI_2,    0.0, 1,  -1.404990, 4.564036, 100.0);
//		mKinematicChain->setDH(5,  0.000,  0.1503,  M_PI_2, M_PI_2, 0,  -3.141593, 3.141593, 100.0);
		mDOF = 5;
	}

	double T0[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			T0[i][j] = 0.0;
	T0[0][0] = 1;
	T0[1][1] = 1;
	T0[2][2] = 1;
	T0[3][3] = 1;
	T0[2][3] = 0.27;
	mKinematicChain->setT0(T0);

	// ready for kinematics
	mKinematicChain->readyForKinematics();

	// variable for ik
	mJacobianPos.Resize(3,mDOF);
	mJacobianDir.Resize(3,mDOF);
	mJacobian.Resize(IK_CONSTRAINTS, mDOF);

	mJacobianDirY.Resize(3,mDOF); mJacobianDirZ.Resize(3,mDOF);
	mJacobian9.Resize(9,mDOF);

	mJointVelLimitsDn.Resize(mDOF);
	mJointVelLimitsUp.Resize(mDOF);

	mKinematicChain->getMaxVel(mJointVelLimitsDn.Array());
	mKinematicChain->getMaxVel(mJointVelLimitsUp.Array());
	mJointVelLimitsDn *= -1.0;

	// Solver init
	mIKSolver.SetSizes(mDOF);  // Dof counts
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);    // One solver with 6 constraints
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver

	MathLib::IndicesVector mJointMapping;
	mJointMapping.clear();
	for(int i=0; i<mDOF; i++){
		mJointMapping.push_back(i);
	}
	mIKSolver.SetDofsIndices(mJointMapping,0); // Joint maps for first solver
}


KatanaKin::KatanaKin(KATANA_MODEL pModel)
{
	mKatanaModel = pModel;
	initiKinematics();
}

KatanaKin::~KatanaKin()
{

}

void KatanaKin::SetJoints(double* pAngles)
{
	mKinematicChain->setJoints(pAngles);
}

void KatanaKin::GetEndPos(double* pPos)
{
	mKinematicChain->getEndPos(pPos);
}

void KatanaKin::GetEndDir(int pIndex, double* pDir)
{
	mKinematicChain->getEndDirAxis(pIndex, pDir);
}

void KatanaKin::GetJoints(double* pAngles)
{
	mKinematicChain->getJoints(pAngles);
}

void KatanaKin::SolveIKWrist(double* pPos, int axis, double* pDir, double dirWeight, double* pAngles, double& posError, double& dirError)
{
	return SolveIK(4, pPos, axis, pDir, dirWeight, pAngles, posError, dirError);
}

void KatanaKin::SolveIKWrist(double* pPos, double* pDirY, double* pDirZ, double dirWeight, double* pAngles, double& posError, double& dirError)
{
	return SolveIK(4, pPos, pDirY, pDirZ, dirWeight, pAngles, posError, dirError);
}

void KatanaKin::SolveIKGripper(double* pPos, int axis, double* pDir, double dirWeight, double* pAngles, double& posError, double& dirError)
{
	return SolveIK(5, pPos, axis, pDir, dirWeight, pAngles, posError, dirError);
}

void KatanaKin::SolveIK(int linkindex, double* pPos, int axis, double* pDir, double dirWeight, double* pAngles, double& posError, double& dirError)
{
	MathLib::Vector lJoints(mDOF);
	MathLib::Vector lPos(3);
	MathLib::Vector lDir(3);
	MathLib::Vector lError(3);
	MathLib::Vector lTarget(6);
	MathLib::Matrix4 lT;


	mIKSolver.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);
	mKinematicChain->getJoints(lJoints.Array());

	for(int frame=0; frame<20; frame++)
	{
		mKinematicChain->getLinkTMatrix(linkindex, lT);
		for(int i=0; i<3; i++)
		{
			lPos(i) = lT(i,3);
			lDir(i) = lT(i, axis);
		}

		lError(0) = (pPos[0] - lPos(0));
		lError(1) = (pPos[1] - lPos(1));
		lError(2) = (pPos[2] - lPos(2));
		posError = lError.Norm();

		lError(0) = (pDir[0] - lDir(0));
		lError(1) = (pDir[1] - lDir(1));
		lError(2) = (pDir[2] - lDir(2));
		dirError = lError.Norm();

		//printf("%lf %lf %lf ,", lPos(0), lPos(1), lPos(2) );
	//	printf("%lf %lf %lf \n", lDir(0), lDir(1), lDir(2) );
		if( (posError < 0.000001) && (dirError*dirWeight < 0.001) ) break;

		mKinematicChain->getJacobianPos(linkindex, mJacobianPos);
		mKinematicChain->getJacobianDirection(linkindex, axis, mJacobianDir);

    	for(int i=0; i<3; i++){
    		mJacobian.SetRow(mJacobianPos.GetRow(i), i  );
    		mJacobian.SetRow(mJacobianDir.GetRow(i), i+3);
    	}
    	mIKSolver.SetJacobian(mJacobian, 0);

    	lTarget(0) = (pPos[0] - lPos(0))/DT;
    	lTarget(1) = (pPos[1] - lPos(1))/DT;
    	lTarget(2) = (pPos[2] - lPos(2))/DT;

    	lTarget(3) = (pDir[0] - lDir(0))/DT*dirWeight;
    	lTarget(4) = (pDir[1] - lDir(1))/DT*dirWeight;
    	lTarget(5) = (pDir[2] - lDir(2))/DT*dirWeight;



		mIKSolver.SetTarget(lTarget, 0);
		mIKSolver.Solve();
		lJoints += mIKSolver.GetOutput()*DT;
		lJoints(5) = 0.0;

		mKinematicChain->setJoints(lJoints.Array());
		mKinematicChain->getJoints(lJoints.Array());
	}

	for(int i=0; i<mDOF; i++) pAngles[i] = lJoints(i);
}
void KatanaKin::SolveIK(int linkindex, double* pPos, double* pDirY, double* pDirZ, double dirWeight, double* pAngles, double& posError, double& dirError)
{
	MathLib::Vector lJoints(mDOF);
	MathLib::Vector lPos(3);
	MathLib::Vector lDir(6);
	MathLib::Vector lError(3);
	MathLib::Vector lTarget(9);
	MathLib::Matrix4 lT;

	mIKSolver.AddSolverItem(9);    // One solver with 6 constraints
	mIKSolver.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);
	mKinematicChain->getJoints(lJoints.Array());

	for(int frame=0; frame<100; frame++)
	{
		mKinematicChain->getLinkTMatrix(linkindex, lT);
		for(int i=0; i<3; i++)
		{
			lPos(i) = lT(i,3);
			lDir(i) = lT(i, 1);
		}
		for(int i=0; i<3; i++)
		{
			lDir(i+3) = lT(i, 2);
		}

		lError(0) = (pPos[0] - lPos(0));
		lError(1) = (pPos[1] - lPos(1));
		lError(2) = (pPos[2] - lPos(2));
		posError = lError.Norm();

		lError(0) = (pDirY[0] - lDir(0));
		lError(1) = (pDirY[1] - lDir(1));
		lError(2) = (pDirY[2] - lDir(2));
		dirError = lError.Norm();

		printf("%lf %lf %lf ,", lPos(0), lPos(1), lPos(2) );
		printf("%lf %lf %lf \n", lDir(0), lDir(1), lDir(2) );
		if( (posError < 0.000001) && (dirError*dirWeight < 0.001) ) break;

		mKinematicChain->getJacobianPos(linkindex, mJacobianPos);
		mKinematicChain->getJacobianDirection(linkindex, 1, mJacobianDirY);
		mKinematicChain->getJacobianDirection(linkindex, 2, mJacobianDirZ);

    	for(int i=0; i<3; i++){
    		mJacobian9.SetRow(mJacobianPos.GetRow(i), i  );
    		mJacobian9.SetRow(mJacobianDirY.GetRow(i), i+3);
    		mJacobian9.SetRow(mJacobianDirZ.GetRow(i), i+6);
    	}
    	mIKSolver.SetJacobian(mJacobian);

    	lTarget(0) = (pPos[0] - lPos(0))/DT;
    	lTarget(1) = (pPos[1] - lPos(1))/DT;
    	lTarget(2) = (pPos[2] - lPos(2))/DT;

    	lTarget(3) = (pDirY[0] - lDir(0))/DT*dirWeight;
    	lTarget(4) = (pDirY[1] - lDir(1))/DT*dirWeight;
    	lTarget(5) = (pDirY[2] - lDir(2))/DT*dirWeight;

    	lTarget(6) = (pDirZ[0] - lDir(3))/DT*dirWeight;
    	lTarget(7) = (pDirZ[1] - lDir(4))/DT*dirWeight;
    	lTarget(8) = (pDirZ[2] - lDir(5))/DT*dirWeight;

    //	cout<<"IK error "<<lTarget.Norm()<<endl;

		mIKSolver.SetTarget(lTarget);
		mIKSolver.Solve();
		lJoints += mIKSolver.GetOutput()*DT;
		lJoints(5) = 0.0;

		mKinematicChain->setJoints(lJoints.Array());
		mKinematicChain->getJoints(lJoints.Array());
	}

	for(int i=0; i<mDOF; i++) pAngles[i] = lJoints(i);
}
