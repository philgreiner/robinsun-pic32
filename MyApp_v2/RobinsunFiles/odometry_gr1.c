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
}


NAMESPACE_CLOSE();