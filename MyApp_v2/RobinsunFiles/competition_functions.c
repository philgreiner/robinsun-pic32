#include "competition_functions.h"
#include "MyApp.h"
#include <math.h>
#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

void calibrate(CtrlStruct *cvs) {
    if(!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) {
        cvs->state->position[1] = (cvs->inputs->team_color)? (1.36) : (-1.36);
        cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);
    }
    if(cvs->inputs->ready_signal || cvs->inputs->start_signal)
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
}

void wait(CtrlStruct *cvs) {
    if(!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) {
        cvs->state->position[1] = (cvs->inputs->team_color)? (1.36) : (-1.36);
        cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);
    }
    if(cvs->inputs->start_signal)
    {
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
        cvs->state->competition_start = (ReadCoreTimer()/(SYS_FREQ/2.0)) - MyMiniProject_tStart ;
    }
}

void blocks_front(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal;
    double d, delta_theta;
    double wheels[2];
    
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BF:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = -0.1;
            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (1.25) : (-1.25);
            cvs->state->goal_position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);

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
            cvs->state->intermediate_goal[0] = -0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.25) : (-1.25);
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);

            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // PUSH BLOC
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5))
                cvs->state->current_action_progress = FORWARD_BF;
            break;
        
        case FORWARD_BF:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            // DEFINE GOAL POSITION
            cvs->state->intermediate_goal[0] = -0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.08) : (-1.08); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-.9*M_PI_2) : (.9*M_PI_2);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // GO TO CLAMP 
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5))
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
                cvs->param->ready_start_astar = 0;
                cvs->state->goal_position[0] = 0.1 ;
                cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (0.5) : (-0.5);
                cvs->state->goal_position[2] = (cvs->inputs->team_color) ? (-1.2*M_PI_2) : (1.2*M_PI_2);
            }
            break;

        case PUSH_BF:
            cvs->state->intermediate_goal[0] = 0.0; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.4) : (-0.4); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-1.1*M_PI_2) : (1.1*M_PI_2);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if (d < 0.05)
            {
                cvs->state->current_action_progress = UNCLAMP_BF;
                cvs->state->errorAngle = 0.0;
                cvs->state->timer = cvs->inputs->t;
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
            if (cvs->inputs->t - cvs->state->timer > 1.5)
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
            
            cvs->state->intermediate_goal[0] = 0.0; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.7) : (-0.7); 
            cvs->state->intermediate_goal[2] = theta;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (delta_theta*180.0/M_PI < 3.5))
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
}

void cabins_close(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal;
    double d, delta_theta;
    double wheels[2];
    
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
            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (1.25) : (-1.25);
            cvs->state->goal_position[2] = 0;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            cvs->outputs->command_blocks = 20;
            
            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_C;
            break;

        case WAIT_FOR_POSITION_C:
            if(!cvs->param->ready_start_astar)
            {
                cvs->state->current_action_progress = BACKWARDS_C;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case BACKWARDS_C:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARDS
            cvs->state->intermediate_goal[0] = -1.3;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.2) : (-1.2);
            cvs->state->intermediate_goal[2] = 0;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            if (cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->current_action_progress = SECOND_C;
                cvs->state->omegaref[R_ID] = 0;
                cvs->state->omegaref[L_ID] = 0;
            }
            break;
        
        case SECOND_C:
            // MOVE TO START FOR SECOND CABIN
            cvs->state->intermediate_goal[0] = -0.5;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.95) : (-1.025);
            cvs->state->intermediate_goal[2] = 0;
            
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 1.5))
            {
                cvs->state->current_action_progress = BACKWARDS_C1;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case BACKWARDS_C1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARDS
            cvs->state->intermediate_goal[0] = -1.3;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.9) : (-0.9);
            cvs->state->intermediate_goal[2] = 0;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            if (cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->current_action_progress = SWITCHES_C;
                cvs->state->omegaref[R_ID] = 0;
                cvs->state->omegaref[L_ID] = 0;
            }
            break;
            
        case SWITCHES_C:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE 
            cvs->state->intermediate_goal[0] = -0.3;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.1) : (-1.1);
            cvs->state->intermediate_goal[2] = theta;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
           
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
          
            // END AFTER ONE SECOND
            if (d < 0.05)
            {
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            }
            break;
    }
}

void blocks_dune_1(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal;
    double d, delta_theta;
    double wheels[2];
    
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
            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (0.57) : (-0.57);
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
            // GO TO TURN IF CLOSE ENOUGH
            if (!cvs->param->ready_start_astar) {
                cvs->state->current_action_progress = TURN_BD1;
            }
            break;
         
        case TURN_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            delta_theta = M_PI - theta;
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : (delta_theta);
            delta_theta = (delta_theta < -M_PI) ? (delta_theta + 2*M_PI) : (delta_theta);
           
            double omega = 2.0*delta_theta;
            cvs->state->omegaref[R_ID] = omega;
            cvs->state->omegaref[L_ID] = -omega;
            
            if(fabs(delta_theta)*180/M_PI < 0.5){
                cvs->state->current_action_progress = FORWARD_BD1;
            }
            break;
            
        case FORWARD_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            cvs->outputs->command_blocks = 0;

            // MOVE FORWARD
            cvs->state->omegaref[R_ID] = 0.75*M_PI;
            cvs->state->omegaref[L_ID] = 0.75*M_PI;

            // GO TO CLAMP 
            if (fabs(-0.72-x) < 0.02) {
                cvs->state->current_action_progress = CLAMP_BD1;
                cvs->state->timer = cvs->inputs->t;
                cvs->state->omegaref[R_ID] = 0;
                cvs->state->omegaref[L_ID] = 0;
            }
            break;
            
        case CLAMP_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = 55.0;

            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 3.5) {
                cvs->state->current_action_progress = MOVE_BACK_BD1;
            }
            break;
            
        case MOVE_BACK_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE BACKWARD
            cvs->state->omegaref[R_ID] = -0.75*M_PI;
            cvs->state->omegaref[L_ID] = -0.75*M_PI;

            // GO TO BRING
            if(fabs(-0.55-x) < 0.02) {
                cvs->state->current_action_progress = POS_BD1;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
            }
            break;
            
        case POS_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            cvs->state->intermediate_goal[0] = -0.35; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.8) : (-0.8); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_4) : (M_PI_4);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5))
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
            
        case BRING_BD1:
            // GO TO UNCLAMP
            if (!cvs->param->ready_start_astar)
                cvs->state->current_action_progress = UNCLAMP_BD1;
            break;
            
        case UNCLAMP_BD1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -30.0;

            // GO TO MOVE_BACK_END
            if (cvs->inputs->t - cvs->state->timer > 3.0)
            {
                cvs->state->current_action_progress = MOVE_BACK_BD1_END;
                cvs->outputs->command_blocks = 0.0;
            }
            break;
            
        case MOVE_BACK_BD1_END:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            cvs->state->intermediate_goal[0] = 0.15; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.8) : (-0.8); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5))
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
            if (cvs->inputs->t - cvs->state->timer > 2)
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
            if (cvs->inputs->t - cvs->state->timer > 2)
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

void fish_catch(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal;
    double d, delta_theta;
    double wheels[2];
    
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->intermediate_goal[0];
    y_goal = cvs->state->intermediate_goal[1];
    theta_goal = cvs->state->intermediate_goal[2];
    
    switch (cvs->state->current_action_progress) {
        case GOTO_FC:
            // SET GOAL POSITION
            cvs->state->goal_position[0] = 0.65;
            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (0.85) : (-0.85);
            cvs->state->goal_position[2] = (cvs->inputs->team_color) ? -M_PI_2 : M_PI_2;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            
            // MAKE SURE CLAMP IS CLOSED
            cvs->outputs->command_blocks = 20;
            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_FC;
            break;
            
        case WAIT_FOR_POSITION_FC:
            d = sqrt((x - cvs->state->goal_position[0])*(x - cvs->state->goal_position[0]) + (y - cvs->state->goal_position[1])*(y - cvs->state->goal_position[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->goal_position[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;

            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 5.0)) {
                cvs->param->ready_start_astar = 0;
                cvs->state->current_action_progress = POSITION_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->intermediate_goal[0] = 0.8;
                cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.3) : (-0.3);
                cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? -M_PI_2 : M_PI_2;
            }
            break;
            
        case POSITION_FC:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5)) {
                cvs->state->current_action_progress = GO_CATCH_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->intermediate_goal[0] = 0.86;
                cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.86) : (-0.86);
                cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? -M_PI_2 : M_PI_2;
            }
            break;
            
        case GO_CATCH_FC:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5)) {
                cvs->state->current_action_progress = BRING_OUT_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
        
        case BRING_OUT_FC:
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_fish_vertical = -20;
            cvs->outputs->command_fish_horizontal = 40;
            if(cvs->inputs->t - cvs->state->timer = 1.5)
            {
                cvs->state->current_action_progress = LOWER_FC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case LOWER_FC:
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_fish_vertical = 20;
            cvs->outputs->command_fish_horizontal = 10;
            if(cvs->inputs->t - cvs->state->timer = 1.5)
            {
                cvs->state->current_action_progress = SWEEP_FC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
    }
}

void parasol_open(CtrlStruct *cvs) {
    // TURN OFF A* and speeds
    cvs->param->ready_start_astar = 0;
    cvs->state->omegaref[R_ID] = 0;
    cvs->state->omegaref[L_ID] = 0;
    cvs->outputs->command_blocks = 0;
    cvs->outputs->command_fish_vertical = 0;
    cvs->outputs->command_fish_horizontal = 0;
    
    // ACTIVATE PARASOL
    if(cvs->inputs->t > 91)
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
    cvs->outputs->command_blocks = 0;
    cvs->outputs->command_fish_vertical = 0;
    cvs->outputs->command_fish_horizontal = 0;
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