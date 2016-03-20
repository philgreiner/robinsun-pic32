/*!
* \file calibrate_gr1.cc
* \brief Function to calibrate before starting time
*/

#include "calibrate_gr1.h"
#include <stdio.h>
#include <math.h>
#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

void calibrate_start(CtrlStruct * cvs)
{
	switch (cvs->state->calibration)
	{
	case CALIBRATEY:
		if (!cvs->inputs->u_switch[R_ID] || !cvs->inputs->u_switch[L_ID])
		{
			cvs->state->omegaref[R_ID] = -2 * M_PI;
			cvs->state->omegaref[L_ID] = -2 * M_PI;
		}
		else
		{
#ifdef SIMU_PROJECT
			cvs->state->position[1] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? -1.502 : 1.502;
			cvs->state->position[2] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? M_PI_2 : -M_PI_2;
#else
			cvs->state->position[1] = -1.502;
			cvs->state->position[2] = M_PI_2;
#endif
			cvs->state->calibration = GOTOY;
		}
		break;

	case GOTOY:
		if (fabs(cvs->state->position[1]) - 1.05 > 0.01)
		{
			cvs->state->omegaref[R_ID] = 2 * M_PI;
			cvs->state->omegaref[L_ID] = 2 * M_PI;
		}
		else
			cvs->state->calibration = TURN;
		break;

	case TURN:
		if (M_PI - fabs(cvs->state->position[2]) > 0.005)
		{
#ifdef SIMU_PROJECT
			cvs->state->omegaref[R_ID] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? 2 * M_PI : -2 * M_PI;
			cvs->state->omegaref[L_ID] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? -2 * M_PI : 2 * M_PI;
#else
			cvs->state->omegaref[R_ID] = 2 * M_PI;
			cvs->state->omegaref[L_ID] = -2 * M_PI;
#endif
		}
		else
			cvs->state->calibration = CALIBRATEX;
		break;
	
	case CALIBRATEX:
		if (!cvs->inputs->u_switch[R_ID] || !cvs->inputs->u_switch[L_ID])
		{
			cvs->state->omegaref[R_ID] = -2 * M_PI;
			cvs->state->omegaref[L_ID] = -2 * M_PI;
		}
		else
		{
			cvs->state->position[0] = 1.002;
			cvs->state->position[2] = M_PI;
			cvs->state->calibration = GOHOMEX;
		}
		break;

	case GOHOMEX: 
		if (cvs->state->position[0] + 0.1 > 0.0)
		{
			cvs->state->omegaref[R_ID] = 2 * M_PI;
			cvs->state->omegaref[L_ID] = 2 * M_PI;
		}
		else
			cvs->state->calibration = GOHOMETURN;
		break;

	case GOHOMETURN:
		if (fabs(M_PI_2 - fabs(cvs->state->position[2])) > 0.01)
		{
#ifdef SIMU_PROJECT
			cvs->state->omegaref[R_ID] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? -2 * M_PI : 2 * M_PI;
			cvs->state->omegaref[L_ID] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? 2 * M_PI : -2 * M_PI;
#else
			cvs->state->omegaref[R_ID] = -2 * M_PI;
			cvs->state->omegaref[L_ID] = 2 * M_PI;
#endif
		}
		else
		{
			cvs->state->calibration = GOHOMEY;
		}
		break;

	case GOHOMEY:
		if (!cvs->inputs->u_switch[R_ID] || !cvs->inputs->u_switch[L_ID])
		{
			cvs->state->omegaref[R_ID] = -2 * M_PI;
			cvs->state->omegaref[L_ID] = -2 * M_PI;
		}
		else
		{
			cvs->state->omegaref[R_ID] = 0;
			cvs->state->omegaref[L_ID] = 0;
#ifdef SIMU_PROJECT
			cvs->state->position[1] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? -1.502 : 1.502;
			cvs->state->position[2] = (cvs->inputs->robot_id == ROBOT_B || cvs->inputs->robot_id == ROBOT_R) ? M_PI_2 : -M_PI_2;
#else
			cvs->state->position[1] = -1.502;
			cvs->state->position[2] = M_PI_2;
#endif
		}
		break;
	}
}

NAMESPACE_CLOSE();