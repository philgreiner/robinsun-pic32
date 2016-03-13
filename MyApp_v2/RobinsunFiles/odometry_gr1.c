/*!
* \file odometry_gr1.cc
* \brief Functions to update position using odometry
*/

#include "odometry_gr1.h"
#include <math.h>

void odometry_estimate(CtrlStruct * cvs)
{
	double deltaT = cvs->inputs->t - cvs->state->lastT;
	double drivenR, drivenL, drivenCenter, angleDiff;

	// Total distance moved for each wheel (30mm = wheel radius)
	drivenR = cvs->inputs->r_wheel_speed * deltaT * 0.03;
	drivenL = cvs->inputs->l_wheel_speed * deltaT * 0.03;
	drivenCenter = (drivenR + drivenL) / 2.0;
	angleDiff = (drivenR - drivenL) / 0.225;

	// We consider the whole movement is done in the new direction
	cvs->state->position_odo[2] = cvs->state->position_odo[2] + angleDiff;
	cvs->state->position_odo[0] = cvs->state->position_odo[0] + drivenCenter*cos(cvs->state->position_odo[2]);
	cvs->state->position_odo[1] = cvs->state->position_odo[1] + drivenCenter*sin(cvs->state->position_odo[2]);
}