/*!
 * \file ctrl_strategy.cc
 * \brief FSM used to feed objectives to subsequent controllers and react to opponent detection
 */

#include "ctrl_strategy_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

void strategy_objective(CtrlStruct *cvs)
{
	double d, x, x_goal, y, y_goal;

	/*	strategy_state features 4 differents states: WAIT_FOR_START, GOTO_OBJ, WAIT_TO_TAKE, WAIT_FOR_DESTINATION
	* A second enum, next_objective, determines the next objective to go to. The table done_objectives determines which objectives have been achieved. (NOTDONE and DONE)
	*	
	*	objectives_on_beacon counts the amount of objectives currently on beacon to go to BASE when full.
	*/
	switch (cvs->state->strategy_state)
	{
	case WAIT_FOR_START:
		cvs->state->goal_position[0] = cvs->state->position_odo[0];
		cvs->state->goal_position[1] = cvs->state->position_odo[1];
		cvs->state->goal_position[2] = cvs->state->position_odo[2];

#ifdef SIMU_PROJECT
		cvs->outputs->flag_release = 0;
#endif

		if (cvs->inputs->t > 0)
		{
			cvs->state->strategy_state = GOTO_OBJ;
		}
		break;

	case GOTO_OBJ: // Select next_objective depending on done_objectives and objectives_on_robot
		objective_selection(cvs);

#ifdef SIMU_PROJECT
		cvs->outputs->flag_release = 0;
#endif
		cvs->state->strategy_state = WAIT_FOR_DESTINATION;
		break;

	case WAIT_TO_TAKE:
		if (cvs->inputs->t - cvs->state->timer > 1.4)
		{
			cvs->state->done_objectives[cvs->state->next_objective] = DONE;
			cvs->state->objectives_on_robot++;
			cvs->state->strategy_state = GOTO_OBJ;
		}
		break;

	case WAIT_FOR_DESTINATION:
		x = cvs->state->position_odo[0];
		y = cvs->state->position_odo[1];

		x_goal = cvs->state->goal_position[0];
		y_goal = cvs->state->goal_position[1];
		d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));

		if (d < 0.02 && cvs->state->next_objective != BASE)
		{
			cvs->state->timer = cvs->inputs->t;
			cvs->state->strategy_state = WAIT_TO_TAKE;
		}
		else if (d < 0.02 && cvs->state->next_objective == BASE)
		{
#ifdef SIMU_PROJECT
			cvs->outputs->flag_release = 1;
#endif
			cvs->state->objectives_on_robot = 0;
			cvs->state->strategy_state = GOTO_OBJ;
		}
		break;

	default:
		printf("Error: unknown state : %d !\n", cvs->state->strategy_state);
		exit(EXIT_FAILURE);
	}
}

void objective_selection(CtrlStruct *cvs)
{
#ifdef SIMU_PROJECT
	printf("Selecting next objective. Objectives on robot: %d\n", cvs->state->objectives_on_robot);
#endif
	if (cvs->state->objectives_on_robot > 1)
	{
#ifdef SIMU_PROJECT
		printf("Robot is full, going back to base.\n");
#endif
		cvs->state->next_objective = BASE;
#ifdef SIMU_PROJECT
		// Set goal position depending on team
		if (cvs->inputs->robot_id == ROBOT_Y || cvs->inputs->robot_id == ROBOT_W)// Team B (green zone)
		{
			cvs->state->goal_position[0] = 0.1;
			cvs->state->goal_position[1] = 0.25;
			cvs->state->goal_position[2] = -M_PI;
		}
		else // Team A (purple zone)
		{
			cvs->state->goal_position[0] = 0.1;
			cvs->state->goal_position[1] = -0.25;
			cvs->state->goal_position[2] = -M_PI;
		}
#else
			cvs->state->goal_position[0] = 0.1;
			cvs->state->goal_position[1] = -0.25;
			cvs->state->goal_position[2] = -M_PI;
#endif
	}
	else
	{
		int i;
		for (i = OBJ0; i <= OBJ6; i++)
		{
#ifdef SIMU_PROJECT
			printf("Objective %d is %d. \n", i, cvs->state->done_objectives[i]);
#endif
			if (cvs->state->done_objectives[i] == NOTDONE)
			{
#ifdef SIMU_PROJECT
				printf("Going to objective %d. \n", i);
#endif
				cvs->state->next_objective = (objectives) i;
				switch (i)
				{
				case OBJ0: 
					cvs->state->goal_position[0] = -0.6;
					cvs->state->goal_position[1] = 0.0;
					cvs->state->goal_position[2] = -M_PI;
					break;
				case OBJ1:
					cvs->state->goal_position[0] = 0.8;
					cvs->state->goal_position[1] = -0.8;
					cvs->state->goal_position[2] = -M_PI;
					break;
				case OBJ2: // OBJ2 and OBJ3 changed
					cvs->state->goal_position[0] = 0.4;
					cvs->state->goal_position[1] = -1.1;
					cvs->state->goal_position[2] = -M_PI;
					break;
				case OBJ3:
					cvs->state->goal_position[0] = 0.8;
					cvs->state->goal_position[1] = 0.8;
					cvs->state->goal_position[2] = -M_PI;
					break;
				case OBJ4:
					cvs->state->goal_position[0] = -0.6;
					cvs->state->goal_position[1] = -1.2;
					cvs->state->goal_position[2] = -M_PI;
					break;
				case OBJ5: 
					cvs->state->goal_position[0] = 0.4;
					cvs->state->goal_position[1] = 1.1;
					cvs->state->goal_position[2] = -M_PI;
					break;
				case OBJ6: 
					cvs->state->goal_position[0] = -0.6;
					cvs->state->goal_position[1] = 1.2;
					cvs->state->goal_position[2] = -M_PI;
					break;
				}
				break;
			}
		}
	}
}


NAMESPACE_CLOSE();

/*

3 points :
OBJ0
cvs->state->goal_position[0] = -0.6;
cvs->state->goal_position[1] = 0.0;
cvs->state->goal_position[2] = -M_PI;

2 points :
OBJ1 (purple side)
cvs->state->goal_position[0] = 0.8;
cvs->state->goal_position[1] = -0.8;
cvs->state->goal_position[2] = -M_PI;
OBJ2 (green side)
cvs->state->goal_position[0] = 0.8;
cvs->state->goal_position[1] = 0.8;
cvs->state->goal_position[2] = -M_PI;

1 point :
OBJ3 (purple side, sea)
cvs->state->goal_position[0] = 0.4;
cvs->state->goal_position[1] = -1.1;
cvs->state->goal_position[2] = -M_PI;
OBJ4 (purple side, sand)
cvs->state->goal_position[0] = -0.6;
cvs->state->goal_position[1] = -1.2;
cvs->state->goal_position[2] = -M_PI;

OBJ5 (green side, sea)
cvs->state->goal_position[0] = 0.4;
cvs->state->goal_position[1] = 1.1;
cvs->state->goal_position[2] = -M_PI;
OBJ6 (green side, sand)
cvs->state->goal_position[0] = -0.6;
cvs->state->goal_position[1] = 1.2;
cvs->state->goal_position[2] = -M_PI;

*/
