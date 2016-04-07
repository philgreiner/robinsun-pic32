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
    GOTO, WAIT_FOR_POSITION, TURN1, FORWARD, CLAMP, PUSH, UNCLAMP, MOVE_BACK
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
        case GOTO:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = 0.092;
            cvs->state->goal_position[1] = -1.162;
            cvs->state->goal_position[2] = M_PI_2;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;

            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION;
            break;

        case WAIT_FOR_POSITION:
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));

            // GO TO TURN IF CLOSE ENOUGH
            if (d < 0.03)
                cvs->state->current_action_progress = TURN1;
            break;

        case TURN1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // Set correct orientation
            double omega = (M_PI_2 - theta);
            cvs->state->omegaref[R_ID] = 2 * M_PI * omega / (fabs(omega));
            cvs->state->omegaref[L_ID] = -2 * M_PI * omega / (fabs(omega));

            // GO TO WAIT 
            if (M_PI_2 - theta < 0.1)
                cvs->state->current_action_progress = FORWARD;
            break;
        
        case FORWARD:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 2 * M_PI;
            cvs->state->omegaref[L_ID] = 2 * M_PI;

            // GO TO WAIT 
            if (0.75 - fabs(y) < 0.05)
            {
                cvs->state->current_action_progress = CLAMP;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
        
        case CLAMP:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = 25.0;

            // GO TO WAIT 
            if (cvs->inputs->t - cvs->state->timer > 2)
                cvs->state->current_action_progress = PUSH;
            break;

        case PUSH:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 2.75 * M_PI;
            cvs->state->omegaref[L_ID] = 2.75 * M_PI;

            // GO TO MOVE_BACK
            if (0.30 - fabs(y) < 0.03)
            {
                cvs->state->current_action_progress = UNCLAMP;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case UNCLAMP:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -25.0;

            // GO TO WAIT 
            if (cvs->inputs->t - cvs->state->timer > 2)
            {
                cvs->state->current_action_progress = MOVE_BACK;
                cvs->outputs->command_blocks = 0.0;
            }
            break;

        case MOVE_BACK:
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

//typedef enum {
//} blocks_dune_1_t;

void blocks_dune_1(CtrlStruct *cvs) {
    cvs->state->objectives[cvs->state->current_objective] = DONE1;
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