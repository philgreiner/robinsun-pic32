/*!
 * \file ctrl_strategy.cc
 * \brief FSM used to feed objectives to subsequent controllers and react to opponent detection
 */

#include "ctrl_strategy_gr1.h"
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
		cvs->state->goal_position[0] = cvs->state->position[0];
		cvs->state->goal_position[1] = cvs->state->position[1];
		cvs->state->goal_position[2] = cvs->state->position[2];

		#ifdef SIMU_PROJECT
			cvs->outputs->flag_release = 0;
		#endif
        
        #ifdef SIMU_PROJECT
            if (cvs->inputs->t > 0.0)
            {
                cvs->state->strategy_state = GOTO_OBJ;
            }
        #else
            if (cvs->inputs->t > 5.0)
            {
                cvs->state->strategy_state = GOTO_OBJ;
            }
        #endif
		break;

	case GOTO_OBJ: // Select next_objective depending on done_objectives and objectives_on_robot
		objective_selection(cvs);
        #ifdef ASTAR
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
        #endif // ASTAR
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
		x = cvs->state->position[0];
		y = cvs->state->position[1];

		x_goal = cvs->state->goal_position[0];
		y_goal = cvs->state->goal_position[1];
		d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));

		double accuracy;
        #ifdef ASTAR
            accuracy = 0.08;
        #endif

        #ifdef POTENTIAL
            accuracy = 0.1;
        #endif

		if (d < accuracy && cvs->state->next_objective != BASE)
		{
            #ifdef SIMU_PROJECT
                cvs->outputs->flag_release = 0;
            #endif
			cvs->state->timer = cvs->inputs->t;
			cvs->state->strategy_state = WAIT_TO_TAKE;
		}
		else if (d < accuracy && cvs->state->next_objective == BASE)
		{
            #ifdef SIMU_PROJECT
                cvs->outputs->flag_release = 1;
            #endif
			cvs->state->objectives_on_robot = 0;
			cvs->state->strategy_state = GOTO_OBJ;
		}
		break;

	default:
        #ifdef SIMU_GAME
            printf("Error: unknown state : %d !\n", cvs->state->strategy_state);
        #endif
		exit(EXIT_FAILURE);
	}
}

void objective_selection(CtrlStruct *cvs)
{
		#ifdef SIMU_PROJECT
			printf("Selecting next objective. Objectives on robot: %d\n", cvs->state->objectives_on_robot);
		#else
		    MyConsole_SendMsg("Selecting Objective.\n");
		#endif

        int timeout = 0;
        #ifdef SIMU_PROJECT
            timeout = (cvs->inputs->t >= 70);
        #else
            timeout = (cvs->inputs->t >= 300);
        #endif
            
		if (cvs->state->objectives_on_robot > 1 || timeout)
		{
            #ifdef SIMU_PROJECT
				printf("Robot is full, going back to base.\n");
            #endif
				cvs->state->next_objective = BASE;

            #ifdef SIMU_PROJECT
                // Set goal position depending on team
                if (cvs->inputs->robot_id == ROBOT_Y || cvs->inputs->robot_id == ROBOT_W)// Team B (green zone)
                {
                    cvs->state->goal_position[0] = 0.05;
                    cvs->state->goal_position[1] = 0.25;
                    cvs->state->goal_position[2] = -M_PI;
                }
                else // Team A (purple zone)
                {
                    cvs->state->goal_position[0] = 0.05;
                    cvs->state->goal_position[1] = -0.25;
                    cvs->state->goal_position[2] = -M_PI;
                }
            #else
				cvs->state->goal_position[0] = 0.1;
				cvs->state->goal_position[1] = -0.25;
				cvs->state->goal_position[2] = -M_PI;
            #endif

            #ifdef ASTAR
				cvs->param->ready_start_astar = 1;
            #endif // ASTAR
        }
        else
        {
            int i;
            for (i = 0; i <= 7; i++)
            {
                if (cvs->state->done_objectives[i] == NOTDONE || i == 7)
                {
                    cvs->state->next_objective = (objectives) i;

                    #ifdef ASTAR
                        cvs->param->ready_start_astar = 1;						// allow astar to calculate its path
                    #endif

                    switch (i)
                    {
                    case OBJ0:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 0. \n");
                        #endif

                        #ifdef ASTAR
                            #ifdef SIMU_GAME
                                cvs->state->goal_position[1] = (cvs->inputs->robot_id == ROBOT_Y || cvs->inputs->robot_id == ROBOT_W) ? 0.0 : 0.2;
                                cvs->state->goal_position[0] = -0.4;
                            #else				
                                cvs->state->goal_position[1] = 0.0;
                                cvs->state->goal_position[0] = -0.6;
                            #endif
                        #else
                            cvs->state->goal_position[1] = 0.0;
                            cvs->state->goal_position[0] = -0.6;
                        #endif

                        cvs->state->goal_position[2] = -M_PI;
                        break;
                    case OBJ1:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 1. \n");
                        #endif
                        cvs->state->goal_position[0] = 0.75;
                        cvs->state->goal_position[1] = -0.8;
                        cvs->state->goal_position[2] = -M_PI;
                        break;
                    case OBJ2:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 2. \n");
                        #endif
                        cvs->state->goal_position[0] = 0.75;
                        cvs->state->goal_position[1] = 0.8;
                        cvs->state->goal_position[2] = -M_PI;
                        break;
                    case OBJ3:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 3. \n");
                        #endif
                        cvs->state->goal_position[0] = 0.4;
                        cvs->state->goal_position[1] = -1.1;
                        cvs->state->goal_position[2] = -M_PI;
                        break;
                    case OBJ4:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 4. \n");
                        #endif
                        cvs->state->goal_position[0] = -0.6;
                        cvs->state->goal_position[1] = -1.15;
                        cvs->state->goal_position[2] = -M_PI;
                        break;
                    case OBJ5:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 5. \n");
                        #endif
                        cvs->state->goal_position[0] = 0.4;
                        cvs->state->goal_position[1] = 1.1;
                        cvs->state->goal_position[2] = -M_PI;
                    break;
                    case OBJ6:
                        #ifdef SIMU_PROJECT
                            printf("Going to objective 6. \n");
                        #endif
                        cvs->state->goal_position[0] = -0.6;
                        cvs->state->goal_position[1] = 1.15;
                        cvs->state->goal_position[2] = -M_PI;
                        break;
                    case BASE:
                        #ifdef SIMU_PROJECT
                            printf("All objectives taken, going back to base.\n");
                            // Set goal position depending on team
                            if (cvs->inputs->robot_id == ROBOT_Y || cvs->inputs->robot_id == ROBOT_W)// Team B (green zone)
                            {
                                cvs->state->goal_position[0] = 0.05;
                                cvs->state->goal_position[1] = 0.25;
                                cvs->state->goal_position[2] = -M_PI;
                            }
                            else // Team A (purple zone)
                            {
                                cvs->state->goal_position[0] = 0.05;
                                cvs->state->goal_position[1] = -0.25;
                                cvs->state->goal_position[2] = -M_PI;
                            }
                        #else
                            cvs->state->goal_position[0] = 0.175;//0.1
                            cvs->state->goal_position[1] = -0.325;//-0.25
                            cvs->state->goal_position[2] = -M_PI;
                        #endif
					break;
				} // SWITCH
				return;
			} // IF
		} // FOR
	} // ELSE
}

NAMESPACE_CLOSE();

/*
3 points :
OBJ0

2 points :
OBJ1 (purple side)
OBJ2 (green side)

1 point :
OBJ3 (purple side, sea)
OBJ4 (purple side, sand)

OBJ5 (green side, sea)
OBJ6 (green side, sand)


________________________________________________________________________________________________________________________________________________________
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|        OBJ4                                                                                                                         OBJ6             |
|                                                                                                                                                      |
|                                                                       OBJ0                                                                           |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                              _____________________________________________________                                                   |
|____________                                 |_________________________  __________________________|                                      ____________|
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|  PURPLE    |                                           BASE           | |        BASE                                                   |   GREEN    |
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|____________|                                                          |_|                                                               |____________|
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|              OBJ3                                                                                                                    OBJ5            |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|__                    OBJ1                                                                                                   OBJ2                   __|
|  \__                                                                                                                                            __/  |
|     \__                                                                                                                                      __/     |
|________\____________________________________________________________________________________________________________________________________/________|

*/