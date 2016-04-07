#include "competition_functions.h"
#include "MyApp.h"
#include <math.h>
#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

//typedef enum {
//} calibrate_t;

void calibrate(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

//typedef enum {
//} wait_t;

void wait(CtrlStruct *cvs) {
    if(cvs->inputs->start_signal)
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

typedef enum {
    GOTO_BF, WAIT_FOR_POSITION_BF, TURN_BF, FORWARD_BF, CLAMP_BF, PUSH_BF, UNCLAMP_BF, MOVE_BACK_BF
} blocks_front_t;

void blocks_front(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BF:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = 0.092;
            cvs->state->goal_position[1] = -1.162;
            cvs->state->goal_position[2] = M_PI_2;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;

            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BF;
            break;

        case WAIT_FOR_POSITION_BF:
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));

            // GO TO TURN IF CLOSE ENOUGH
            if (d < 0.03)
                cvs->state->current_action_progress = TURN_BF;
            break;

        case TURN_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // Set correct orientation
            double omega = (M_PI_2 - theta);
            cvs->state->omegaref[R_ID] = 2 * M_PI * omega / (fabs(omega));
            cvs->state->omegaref[L_ID] = -2 * M_PI * omega / (fabs(omega));

            // GO TO FORWARD 
            if (M_PI_2 - theta < 0.1)
                cvs->state->current_action_progress = FORWARD_BF;
            break;
        
        case FORWARD_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 2 * M_PI;
            cvs->state->omegaref[L_ID] = 2 * M_PI;

            // GO TO CLAMP 
            if (0.75 - fabs(y) < 0.05)
            {
                cvs->state->current_action_progress = CLAMP_BF;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
        
        case CLAMP_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = 25.0;

            // GO TO PUSH 
            if (cvs->inputs->t - cvs->state->timer > 2)
                cvs->state->current_action_progress = PUSH_BF;
            break;

        case PUSH_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 2.75 * M_PI;
            cvs->state->omegaref[L_ID] = 2.75 * M_PI;

            // GO TO UNCLAMP
            if (0.30 - fabs(y) < 0.03)
            {
                cvs->state->current_action_progress = UNCLAMP_BF;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case UNCLAMP_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -25.0;

            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 2)
            {
                cvs->state->current_action_progress = MOVE_BACK_BF;
                cvs->outputs->command_blocks = 0.0;
            }
            break;

        case MOVE_BACK_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            MyCyclone_Write(A_PICtoFPGA, 0x0001 & 1);

            // MOVE BACKWARD
            cvs->state->omegaref[R_ID] = -2.75 * M_PI;
            cvs->state->omegaref[L_ID] = -2.75 * M_PI;

            // ACTION IS DONE
            if (0.60 - fabs(y) < 0.03)
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
}

//typedef enum {
//} cabins_close_t;

void cabins_close(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

typedef enum {GOTO_BD1, WAIT_FOR_POSITION_BD1, TURN_BD1, FORWARD_BD1, CLAMP_BD1, MOVE_BACK_BD1, BRING_BD1, UNCLAMP_BD1, MOVE_BACK_BD1_END
} blocks_dune_1_t;

void blocks_dune_1(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BD1:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = -0.5;
            cvs->state->goal_position[1] = 0;
            cvs->state->goal_position[2] = M_PI;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            
            // START OPENING CLAMP
            cvs->outputs->command_blocks = -20;

            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BD1;
            break;
            
        case WAIT_FOR_POSITION_BD1:
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));
            
            // STOP MOTOR IF OPEN
            if(cvs->state->clamp == OPEN)
                cvs->outputs->command_blocks = 0;

            // GO TO TURN IF CLOSE ENOUGH
            if (d < 0.03)
                cvs->state->current_action_progress = TURN_BD1;
            break;
            
        case TURN_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // Set correct orientation
            double omega = (M_PI - theta);
            if (omega > M_PI) omega -= 2 * M_PI;
            if (omega < -M_PI) omega += 2 * M_PI;
            
            cvs->state->omegaref[R_ID] = 2 * M_PI * omega / (fabs(omega));
            cvs->state->omegaref[L_ID] = -2 * M_PI * omega / (fabs(omega));

            // GO TO FORWARD 
            if (M_PI_2 - theta < 0.1)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = FORWARD_BD1;
            }
            break;
            
        case FORWARD_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 2 * M_PI;
            cvs->state->omegaref[L_ID] = 2 * M_PI;

            // GO TO CLAMP 
            if (-0.328 - y < 0.01 | (cvs->inputs->odo_l_speed < M_PI && (cvs->inputs->t - cvs->state->timer) > 1))
            {
                cvs->state->current_action_progress = CLAMP_BD1;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CLAMP_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = 25.0;

            // GO TO MOVE_BACK 
            if (cvs->state->clamp == CLAMPED | (cvs->inputs->t - cvs->state->timer > 2))
            {
                cvs->state->current_action_progress = PUSH_BF;
            }
            break;
            
        case MOVE_BACK_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARD
            cvs->state->omegaref[R_ID] = -2.75 * M_PI;
            cvs->state->omegaref[L_ID] = -2.75 * M_PI;

            // GO TO BRING
            if (-0.2 - y < 0.03)
                cvs->state->current_action_progress = BRING_BD1;
            break;
            
        case BRING_BD1:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = 0.0;
            cvs->state->goal_position[1] = -0.2;
            cvs->state->goal_position[2] = M_PI;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));
            
            // GO TO UNCLAMP
            if(d < 0.05)
                cvs->state->current_action_progress = UNCLAMP_BD1;
            break;
            
        case UNCLAMP_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -25.0;

            // GO TO MOVE_BACK_END
            if (cvs->state->clamp == UNCLAMPED | (cvs->inputs->t - cvs->state->timer > 2))
            {
                cvs->state->current_action_progress = MOVE_BACK_BD1_END;
            }
            break;
            
        case MOVE_BACK_BD1_END:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARD
            cvs->state->omegaref[R_ID] = -2.75 * M_PI;
            cvs->state->omegaref[L_ID] = -2.75 * M_PI;

            // ACTION IS DONE
            if (0.60 - fabs(y) < 0.03)
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
}

//typedef enum {
//} blocks_dune_2_t;

void blocks_dune_2(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

//typedef enum {
//} blocks_cabins_t;

void blocks_cabins(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

//typedef enum {
//} fish_catch_t;

void fish_catch(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

void parasol_open(CtrlStruct *cvs) {
    // TURN OFF A* and speeds
    cvs->param->ready_start_astar = 0;
    cvs->state->omegaref[R_ID] = 0;
    cvs->state->omegaref[L_ID] = 0;
    
    // ACTIVATE PARASOL
    MyCyclone_Write(A_PICtoFPGA, 0x0001 & 1);

    // MARK AS DONE
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

//typedef enum {
//} stop_end_t;

void stop_end(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

NAMESPACE_CLOSE();

/*
________________________________________________________________________________________________________________________________________________________
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
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
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|__                                                                                                                                                  __|
|  \__                                                                                                                                            __/  |
|     \__                                                                                                                                      __/     |
|________\____________________________________________________________________________________________________________________________________/________|

 */