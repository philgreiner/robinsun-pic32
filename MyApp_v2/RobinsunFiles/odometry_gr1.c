/*!
* \file odometry_gr1.cc
* \brief Functions to update position using odometry
*/

#include "odometry_gr1.h"

NAMESPACE_INIT(ctrlGr1); 

void odometry_estimate(CtrlStruct * cvs)
{
	double deltaT = cvs->inputs->t - cvs->state->lastT;
	double drivenR, drivenL, R, b, drivenCenter, drivenCenter2, angleDiff, angleDiff2;
	int i,j;

	R = 0.03;	// Wheel radius [m]
	b = 0.225;	// Distance between left and right wheels [m]

	// Total distance moved for each wheel (30mm = wheel radius)
	drivenR = cvs->inputs->r_wheel_speed * deltaT * R;
	drivenL = cvs->inputs->l_wheel_speed * deltaT * R;
	drivenCenter = (drivenR + drivenL) / 2.0;
	angleDiff = (drivenR - drivenL) / b;

	// We consider the whole movement is done in the new direction
	// Update of the mean
	cvs->state->position_odo[2] = cvs->state->position_odo[2] + angleDiff;
	cvs->state->position_odo[0] = cvs->state->position_odo[0] + drivenCenter*cos(cvs->state->position_odo[2]);
	cvs->state->position_odo[1] = cvs->state->position_odo[1] + drivenCenter*sin(cvs->state->position_odo[2]);

	/* ------------------------ Kalman Filter ------------------------ */
	
	angleDiff2 = cvs->state->position_odo[2] - angleDiff/2.0;
	drivenCenter2 = drivenCenter/(2.0*b);

	// Jacobian of f(x,u) wrt x and its transposed
	double Fx[3][3] = {{1.0, 0.0,-drivenCenter*sin(angleDiff2)},
	      		 	   {0.0, 1.0, drivenCenter*cos(angleDiff2)},
	      		 	   {0.0, 0.0, 1.0						 }};

	double Fx_transposed [3][3];
	for (i=0;i<3;i=i+1)
		for (j=0;j<3;j=j+1)
			Fx_transposed[i][j] = Fx[j][i];

	// Jacobian of f(x,u) wrt u and its transposed
	double Fu[3][2] = {{cos(angleDiff2)/2.0 - drivenCenter2*sin(angleDiff2) , cos(angleDiff2)/2.0 + drivenCenter2*sin(angleDiff2)},
		  		 	   {sin(angleDiff2)/2.0 + drivenCenter2*cos(angleDiff2) , sin(angleDiff2)/2.0 - drivenCenter2*cos(angleDiff2)},
		  		 	   {1.0/b 											    , -1.0/b 											}};

	double Fu_transposed[2][3];
	for(i=0;i<2;i=i+1)
		for(j=0;j<3;j=j+1)
			Fu_transposed[i][j] = Fu[j][i];

	// Covariance of the noise associated to the motion model
	double Q[2][2] = {{cvs->param->kr * fabs(drivenR),0.0 							   },
		 			  {0.0 							 ,cvs->param->kl * fabs(drivenL)}};

	// P_est = Fx * P * Fx' + Fu * Q * Fu'

	for(i=0;i<3;i=i+1)
		for(j=0;j<3;j=j+1)
			cvs->state->covariance_odo[i][j] = Fx[i][j];					// TO BE MODIFIED WHEN MULTIPLICATION IS OK
	// cvs->state->covariance_odo = Fx * cvs->state->covariance * Fx_transposed + Fu * Q * Fu_transposed;
}


NAMESPACE_CLOSE();