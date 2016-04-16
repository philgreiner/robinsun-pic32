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
    MyConsole_SendMsg("Calibrate set to DONE1.\n");
}

//typedef enum {
//} wait_t;

void wait(CtrlStruct *cvs) {
    if(cvs->inputs->start_signal)
    {
        MyConsole_SendMsg("Start message received.\n");
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
    }
}

void blocks_front(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d, wheels[2], dest[3];
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BF:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = 0.5;//-0.1;
            cvs->state->goal_position[1] = 0.75;//-1.25;
            cvs->state->goal_position[2] = 3*M_PI_4;//M_PI_2

            // ACTIVATE A*
            cvs->param->ready_start_astar = 0;
            cvs->param->Astar_path_active = 0;
            
            // OPEN CLAMPS
            cvs->outputs->command_blocks = -20;

            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BF;
            break;

        case WAIT_FOR_POSITION_BF:
            // COMPUTE REMAINING DISTANCE
            dest[0] = 0.5;//-0.1; 
            dest[1] = 0.75;//-1.25; 
            dest[2] = 3*M_PI_4;//M_PI_2;
            d = sqrt((x - dest[0])*(x - dest[0]) + (y - dest[1])*(y - dest[1]));
            
            gotoPoint(cvs,dest,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // PUSH BLOC
            if ((d < 0.05) && (fabs(cvs->state->position[2] - dest[2])*180.0/M_PI < 2.5))
                cvs->state->current_action_progress = FORWARD_BF;
            break;
        
        case FORWARD_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            // DEFINE GOAL POSITION
            dest[0] = -0.1; 
            dest[1] = -1.1; 
            dest[2] = M_PI_2;
            d = sqrt((x - dest[0])*(x - dest[0]) + (y - dest[1])*(y - dest[1]));
            
            gotoPoint(cvs,dest,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // GO TO CLAMP 
            if ((d < 0.05) && (fabs(cvs->state->position[2] - dest[2])*180.0/M_PI < 2.5))
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
            
            cvs->outputs->command_blocks = 45.0;

            // GO TO PUSH 
            if (cvs->inputs->t - cvs->state->timer > 2)
            {
                cvs->state->current_action_progress = PUSH_BF;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->param->ready_start_astar = 1;
                cvs->state->goal_position[0] = 0.1 ;
                cvs->state->goal_position[1] = -0.5;
                cvs->state->goal_position[2] = M_PI_2;
            }
            break;

        case PUSH_BF:
            // GO TO UNCLAMP
            if (!cvs->param->ready_start_astar)
            {
                cvs->state->current_action_progress = UNCLAMP_BF;
                cvs->state->errorAngle = 0.0;
            }
            break;
            
        case UNCLAMP_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -45.0;

            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 2)
            {
                cvs->state->current_action_progress = MOVE_BACK_BF;
                cvs->outputs->command_blocks = 0.0;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
            }
            break;

        case MOVE_BACK_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            dest[0] = 0.1; 
            dest[1] = -0.75; 
            dest[2] = 0;
            d = sqrt((x - dest[0])*(x - dest[0]) + (y - dest[1])*(y - dest[1]));
            
            gotoPoint(cvs,dest,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(cvs->state->position[2] - dest[2])*180.0/M_PI < 2.5))
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
}

void cabins_close(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d, wheels[2], dest[3];
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];
    
    switch(cvs->state->current_action_progress) {
        case GOTO_C:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = -0.5;
            cvs->state->goal_position[1] = -1.1;
            cvs->state->goal_position[2] = M_PI;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            cvs->outputs->command_blocks = 15;
            
            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_C;
            break;

        case WAIT_FOR_POSITION_C:
            // MOVE BACKWARDS IF CLOSE ENOUGH
            if (!cvs->param->ready_start_astar)
                cvs->state->current_action_progress = BACKWARDS_C;
            break;
            
        case BACKWARDS_C:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARDS
            cvs->state->omegaref[R_ID] = -2 * M_PI;
            cvs->state->omegaref[L_ID] = -2 * M_PI;

            // GO TO CLAMP 
            if (x + 0.875 < 0.02)//cvs->inputs->u_switch[R_ID] & cvs->inputs->u_switch[L_ID])
            {
                cvs->state->current_action_progress = SWITCHES_C;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case SWITCHES_C:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARDS
            cvs->state->omegaref[R_ID] = M_PI;
            cvs->state->omegaref[L_ID] = M_PI;

            // END AFTER ONE SECOND
            if(cvs->inputs->t - cvs->state->timer > 1)
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
}


void blocks_dune_1(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d, dest[3], wheels[2];
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
            cvs->state->goal_position[1] = -0.5;
            cvs->state->goal_position[2] = M_PI;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            
            // START OPENING CLAMP
            cvs->outputs->command_blocks = 0;

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
            if (!cvs->param->ready_start_astar)
                cvs->state->current_action_progress = TURN_BD1;
            break;

        case TURN_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            cvs->outputs->command_blocks = -20;
            // Set correct orientation
//            double omega = (M_PI - theta);
//            if (omega > M_PI) omega -= 2 * M_PI;
//            if (omega < -M_PI) omega += 2 * M_PI;
//            
//            cvs->state->omegaref[R_ID] = 2 * M_PI * omega / (fabs(omega));
//            cvs->state->omegaref[L_ID] = -2 * M_PI * omega / (fabs(omega));
            dest[0] = x; dest[1] = y; dest[2] = M_PI;
//            // Set correct orientation
//            double omega = (M_PI - theta);
//            if (omega > M_PI) omega -= 2 * M_PI;
//            if (omega < -M_PI) omega += 2 * M_PI;
            gotoPoint(cvs,dest,wheels);
//            cvs->state->omegaref[R_ID] = M_PI_4 * omega / (fabs(omega));
//            cvs->state->omegaref[L_ID] = -M_PI_4 * omega / (fabs(omega));
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            // GO TO FORWARD 
            if (fabs(M_PI - theta)*180/M_PI < 3.0)
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
                cvs->state->current_action_progress = MOVE_BACK_BD1;
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

void blocks_cabins(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BC:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = -0.5;
            cvs->state->goal_position[1] = 0.65;
            cvs->state->goal_position[2] = M_PI;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            
            // START OPENING CLAMP
            cvs->outputs->command_blocks = -20;

            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BC;
            break;
            
        case WAIT_FOR_POSITION_BC:
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - x_goal)*(x - x_goal) + (y - y_goal)*(y - y_goal));
            
            // STOP MOTOR IF OPEN
            if(cvs->state->clamp == OPEN)
                cvs->outputs->command_blocks = 0;

            // GO TO TURN IF CLOSE ENOUGH
            if (cvs->param->ready_start_astar)
                cvs->state->current_action_progress = TURN_BC;
            break;
            
        case TURN_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // Set correct orientation
            double omega = (M_PI - theta);
            if (omega > M_PI) omega -= 2 * M_PI;
            if (omega < -M_PI) omega += 2 * M_PI;
            
            cvs->state->omegaref[R_ID] = 2 * M_PI * omega / (fabs(omega));
            cvs->state->omegaref[L_ID] = -2 * M_PI * omega / (fabs(omega));

            // GO TO FORWARD 
            if (omega < 0.1)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = FORWARD_BC;
            }
            break;
            
        case FORWARD_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 2 * M_PI;
            cvs->state->omegaref[L_ID] = 2 * M_PI;

            // GO TO CLAMP 
            if (-0.27 - y < 0.01 | (cvs->inputs->odo_l_speed < M_PI && (cvs->inputs->t - cvs->state->timer) > 1))
            {
                cvs->state->current_action_progress = CLAMP_BC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CLAMP_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = 25.0;

            // GO TO MOVE_BACK 
            if (cvs->state->clamp == CLAMPED | (cvs->inputs->t - cvs->state->timer > 2))
            {
                cvs->state->current_action_progress = BACKWARDS_BC;
            }
            break;
            
        case BACKWARDS_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARD
            cvs->state->omegaref[R_ID] = -2.75 * M_PI;
            cvs->state->omegaref[L_ID] = -2.75 * M_PI;

            // GO TO BRING
            if (-0.2 - y < 0.03)
                cvs->state->current_action_progress = BRING_BC;
            break;

        case BRING_BC:
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
                cvs->state->current_action_progress = UNCLAMP_BC;
            break;
            
        case UNCLAMP_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -25.0;

            // GO TO MOVE_BACK_END
            if (cvs->state->clamp == UNCLAMPED | (cvs->inputs->t - cvs->state->timer > 2))
            {
                cvs->state->current_action_progress = MOVE_BACK_BC;
            }
            break;
            
        case MOVE_BACK_BC:
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
    if(cvs->inputs->t > 90)
    {
        MyCyclone_Write(A_PICtoFPGA, 0x0001 & 1);
        // MARK AS DONE
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
    }
}

//typedef enum {
//} stop_end_t;

void stop_end(CtrlStruct *cvs) {
    cvs->state->omegaref[R_ID] = 0;
    cvs->state->omegaref[L_ID] = 0;
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