#include "competition_functions.h"
#include "MyApp.h"
#include <math.h>
#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

void calibrate(CtrlStruct *cvs) {
    brakesWheels((unsigned int) 1);
    if(cvs->inputs->ready_signal || cvs->inputs->start_signal)
    {
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
        cvs->state->position[0] = (cvs->inputs->team_color) ? (-0.09) : (-0.21);
        cvs->state->position[1] = (cvs->inputs->team_color) ? (1.32) : (-1.32);
        cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI) : (0);
        
        cvs->state->last_astarPos[0] = -0.1;
        cvs->state->last_astarPos[1] = (cvs->inputs->team_color) ? (1.25) : (-1.25);
        if(cvs->inputs->strategy_cabins) 
        {
            cvs->state->objectives[CABINS] = NOTDONE1;
            cvs->state->objectives[FISH] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_1] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_2] = DONE1;
            cvs->state->objectives[BLOCKS_CABINS] = DONE1;
            cvs->state->objectives[BLOCKS_FRONT] = DONE1;
        }
        if(cvs->inputs->strategy_fish)
        {
            cvs->state->objectives[CABINS] = DONE1;
            cvs->state->objectives[FISH] = NOTDONE1;
            cvs->state->objectives[BLOCKS_DUNE_1] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_2] = DONE1;
            cvs->state->objectives[BLOCKS_CABINS] = DONE1;
            cvs->state->objectives[BLOCKS_FRONT] = DONE1;
        }
        if(cvs->inputs->strategy_blockscabins)
        {
            cvs->state->objectives[CABINS] = DONE1;
            cvs->state->objectives[FISH] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_1] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_2] = DONE1;
            cvs->state->objectives[BLOCKS_CABINS] = NOTDONE1;
            cvs->state->objectives[BLOCKS_FRONT] = DONE1;
        }
        if(cvs->inputs->mobility)
        {
            cvs->state->objectives[CABINS] = DONE1;
            cvs->state->objectives[FISH] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_1] = DONE1;
            cvs->state->objectives[BLOCKS_DUNE_2] = NOTDONE1;
            cvs->state->objectives[BLOCKS_CABINS] = DONE1;
            cvs->state->objectives[BLOCKS_FRONT] = DONE1;
        }
    }
}

void wait(CtrlStruct *cvs) {
    if(!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) {
        cvs->state->position[1] = (cvs->inputs->team_color)? (1.36) : (-1.36);
        cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);
    }
    if(cvs->inputs->start_signal)
    {
        cvs->state->objectives[cvs->state->current_objective] = DONE1;
        MyMiniProject_tStart += cvs->inputs->t;
        cvs->state->lastT = 0.0;
        cvs->inputs->t = 0.0;
        brakesWheels((unsigned int) 0);
        if(cvs->inputs->mobility)
            cvs->state->current_objective = BLOCKS_DUNE_2;
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
            
            // GO TO WAIT FOR DESTINATION
            cvs->state->timer = cvs->inputs->t;
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BF;
            break;

        case WAIT_FOR_POSITION_BF:
            // COMPUTE REMAINING DISTANCE
            cvs->state->intermediate_goal[0] = -0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.28) : (-1.25);
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);

            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            
            // OPEN CLAMPS
            if(cvs->inputs->t - cvs->state->timer > 0.5)
                cvs->outputs->command_blocks = -30;

            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // PUSH BLOCK
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 5))
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
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 5))
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
            if (cvs->inputs->t - cvs->state->timer > 1)
            {
                cvs->state->current_action_progress = PUSH_BF;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;

        case PUSH_BF:
            cvs->state->intermediate_goal[0] = 0.0; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.38) : (-0.38); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-1.1*M_PI_2) : (1.1*M_PI_2);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // ACTION IS DONE
            if (d < 0.05 || (cvs->inputs->t-cvs->state->timer > 4))
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
            if (cvs->inputs->t - cvs->state->timer > 1)
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
            
            cvs->state->intermediate_goal[0] = -0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.1) : (-1.1); 
            cvs->state->intermediate_goal[2] = theta;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (delta_theta*180.0/M_PI < 2))
            {
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
                cvs->state->errorIntR = 0.0;
                cvs->state->errorIntL = 0.0;
            }
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
//            cvs->state->goal_position[0] = -0.5;
//            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (1.2) : (-1.2);
//            cvs->state->goal_position[2] = 0;
            cvs->state->intermediate_goal[0] = -0.5;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.2) : (-1.2);
            cvs->state->intermediate_goal[2] = 0;
            // ACTIVATE A*
            cvs->param->ready_start_astar = 0;
            cvs->param->Astar_path_active = 0;
            cvs->outputs->command_blocks = 20;
            
            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_C;
            break;

        case WAIT_FOR_POSITION_C:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (delta_theta*180.0/M_PI < 2)) {
                cvs->state->current_action_progress = BACKWARDS_C;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case BACKWARDS_C:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            cvs->param->gotoPointSpeed = 1;
            // MOVE BACKWARDS
            cvs->state->intermediate_goal[0] = -1.05;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.2) : (-1.15);
            cvs->state->intermediate_goal[2] = 0;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            if ((cvs->inputs->t - cvs->state->timer > 1.5 && x < -0.84) || cvs->inputs->t - cvs->state->timer > 2.7)
            {
                cvs->state->current_action_progress = SECOND_C_FRONT;
                cvs->state->omegaref[R_ID] = 0;
                cvs->state->omegaref[L_ID] = 0;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->param->gotoPointSpeed = 0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case SECOND_C_FRONT:
                cvs->state->omegaref[R_ID] = 4*M_PI;
                cvs->state->omegaref[L_ID] = 4*M_PI;
                if(cvs->inputs->t - cvs->state->timer > 1)
                    cvs->state->current_action_progress = SECOND_C;
            break;
            
        case SECOND_C:
            // MOVE TO START FOR SECOND CABIN
            cvs->state->intermediate_goal[0] = -0.5;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.05) : (-1.05);
            cvs->state->intermediate_goal[2] = -10*M_PI/180;
            
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3))
            {
                cvs->state->current_action_progress = BACKWARDS_C1;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case BACKWARDS_C1:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            cvs->param->gotoPointSpeed = 1;

            // MOVE BACKWARDS
            cvs->state->intermediate_goal[0] = -1.05;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.95) : (-0.9);
            cvs->state->intermediate_goal[2] = 0;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            if ((cvs->inputs->t - cvs->state->timer > 1.5 && x < -0.84) || cvs->inputs->t - cvs->state->timer > 2.7)
            {
                cvs->state->current_action_progress = SWITCHES_C;
                cvs->state->omegaref[R_ID] = 0;
                cvs->state->omegaref[L_ID] = 0;
                cvs->state->timer = cvs->inputs->t;
                cvs->param->gotoPointSpeed = 0;
            }
            break;
            
        case SWITCHES_C:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // MOVE 
            cvs->state->intermediate_goal[0] = -0.6;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.95) : (-0.95);
            cvs->state->intermediate_goal[2] = theta;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
           
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = 2*M_PI; //wheels[R_ID];
            cvs->state->omegaref[L_ID] = 2*M_PI; //wheels[L_ID];
          
            // END AFTER ONE SECOND
            if (d < 0.05 || cvs->inputs->t - cvs->state->timer > 2)
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
            cvs->state->goal_position[1] = 0.0;
            cvs->state->goal_position[2] = M_PI;

            // ACTIVATE A*
            cvs->param->ready_start_astar = 1;
            cvs->param->Astar_path_active = 0;
            
            // CLOSE CLAMP
            cvs->outputs->command_blocks = 20;

            // GO TO WAIT FOR DESTINATION
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BD1;
            break;
            
        case WAIT_FOR_POSITION_BD1:
            // GO TO TURN IF CLOSE ENOUGH
            if (!cvs->param->ready_start_astar) {
                cvs->state->current_action_progress = CALIBRATE_BD1;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
         
        case CALIBRATE_BD1:
            // OPEN CLAMP
            cvs->outputs->command_blocks = -40;

            cvs->state->omegaref[R_ID] = -2*M_PI;
            cvs->state->omegaref[L_ID] = -2*M_PI;
            
            if((fabs(cvs->inputs->odo_r_speed - cvs->inputs->r_wheel_speed) > 0.5*M_PI && fabs(cvs->inputs->odo_l_speed - cvs->inputs->l_wheel_speed) > 0.5*M_PI && (cvs->inputs->t - cvs->state->timer > 2)) || cvs->inputs->t - cvs->state->timer > 3) {
                cvs->state->current_action_progress = FORWARD_BD1;
                cvs->state->position[2] = M_PI;
                cvs->state->position[0] = -0.39;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case FORWARD_BD1:
            cvs->outputs->command_blocks = -20;

            cvs->param->gotoPointSpeed = 1;
            cvs->state->intermediate_goal[0] = -0.7; 
            cvs->state->intermediate_goal[1] = 0.0; 
            cvs->state->intermediate_goal[2] = M_PI;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // GO TO CLAMP 
            if ((fabs(cvs->inputs->odo_r_speed - cvs->inputs->r_wheel_speed) > 0.5*M_PI && fabs(cvs->inputs->odo_l_speed - cvs->inputs->l_wheel_speed) > 0.5*M_PI && (cvs->inputs->t - cvs->state->timer > 0.7)) || d < 0.02 || cvs->inputs->t - cvs->state->timer > 3) {
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
            
            cvs->outputs->command_blocks = 65.0;

            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 1.5) {
                cvs->state->current_action_progress = MOVE_BACK_BD1;
            }
            break;
            
        case MOVE_BACK_BD1:
            cvs->param->gotoPointSpeed = 1;
            cvs->state->intermediate_goal[0] = -0.5; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? 1.3 : -1.3; 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_4) : (M_PI_4);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5)) {
                cvs->state->current_action_progress = POS_BD1;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
            }
            break;
            
        case POS_BD1:
            // TURN OFF A*
//            cvs->param->ready_start_astar = 1;
//            cvs->param->Astar_path_active = 0;
            
//            cvs->state->goal_position[0] = 0.1;
//            cvs->state->goal_position[1] = cvs->inputs->team_color ? 0.5 : -0.5;
//            cvs->state->goal_position[2] = cvs->inputs->team_color ? -M_PI_2 : M_PI_2;

            cvs->state->intermediate_goal[0] = -0.35; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.2) : (-1.2); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_4) : (M_PI_4);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 5))
                cvs->state->current_action_progress = BRING_BD1;
            break;
            
        case BRING_BD1:
            // GO TO UNCLAMP
            cvs->state->intermediate_goal[0] = 0.1;
            cvs->state->intermediate_goal[1] = cvs->inputs->team_color ? 0.5 : -0.5;
            cvs->state->intermediate_goal[2] = cvs->inputs->team_color ? -M_PI_2 : M_PI_2;
            
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5))
//            if (!cvs->param->ready_start_astar)
                cvs->state->current_action_progress = UNCLAMP_BD1;
            break;
            
        case UNCLAMP_BD1:
            // TURN OFF A*
            cvs->param->gotoPointSpeed = 0;
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -30.0;

            // GO TO MOVE_BACK_END
            if (cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->current_action_progress = MOVE_BACK_BD1_END;
                cvs->outputs->command_blocks = 0.0;
            }
            break;
            
        case MOVE_BACK_BD1_END:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            cvs->state->intermediate_goal[0] = 0.15; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.7) : (-0.7); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if ((d < 0.07) && (fabs(delta_theta)*180.0/M_PI < 7))
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
}

//typedef enum {
//} blocks_dune_2_t;

void blocks_dune_2(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d, delta_theta, wheels[2];
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BCo:
            // SET GOAL POSITION
            cvs->state->intermediate_goal[0] = -0.5;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.2) : (-1.2);
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);

            // ACTIVATE A*
//            cvs->param->ready_start_astar = 1;
//            cvs->param->Astar_path_active = 0;

            // GO TO WAIT FOR DESTINATION
            cvs->state->timer = cvs->inputs->t;
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BCo;
            break;
            
        case WAIT_FOR_POSITION_BCo:
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            cvs->param->gotoPointSpeed = 0;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
//            if (!cvs->param->ready_start_astar) {
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3) || cvs->inputs->t - cvs->state->timer > 10)
            // GO TO TURN IF CLOSE ENOUGH
//            if (!cvs->param->ready_start_astar)
            {
                cvs->state->current_action_progress = CALIBRATE_BCo;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CALIBRATE_BCo:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            cvs->state->intermediate_goal[0] = -0.5; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.5) : (-1.5); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = -2.5*M_PI;
            cvs->state->omegaref[L_ID] = -2.5*M_PI;
            
            // GO TO FORWARD 
            if ((!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) || cvs->inputs->t - cvs->state->timer > 1.8)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = POSITION_BCo;
                cvs->state->position[1] = (cvs->inputs->team_color) ? (1.36) : (-1.36); 
                cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2); 
            }
            break;
            
        case POSITION_BCo:
            cvs->param->gotoPointSpeed = 0;

            cvs->state->intermediate_goal[0] = -0.5; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (-0.55) : (0.54); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? M_PI : -M_PI; 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            // START OPENING CLAMP
            cvs->outputs->command_blocks = -30;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            if (((d < 0.03) && (fabs(delta_theta)*180.0/M_PI < 2)) || cvs->inputs->t - cvs->state->timer > 10) {
                cvs->state->current_action_progress = CALIBRATE_X_BCo;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CALIBRATE_X_BCo:
            cvs->state->omegaref[R_ID] = -2*M_PI;
            cvs->state->omegaref[L_ID] = -2*M_PI;
            
            // GO TO FORWARD 
            if ((!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) || cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = FORWARD_BCo;
                cvs->state->position[0] = -0.39; 
                cvs->state->position[2] = -M_PI; 
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
            }
            break;
            
        case FORWARD_BCo:
            // TURN OFF A*
            cvs->state->intermediate_goal[0] = -0.79; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (-0.54) : (0.53); 
            cvs->state->intermediate_goal[2] = M_PI; 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            cvs->param->gotoPointSpeed = 1;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // GO TO CLAMP 
            if ((d < 0.05) || (cvs->inputs->t - cvs->state->timer > 3.0))
            {
                cvs->state->current_action_progress = CLAMP_BCo;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CLAMP_BCo:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = -0.5*M_PI;
            cvs->state->omegaref[L_ID] = -0.5*M_PI;
            
            cvs->outputs->command_blocks = 75.0;

            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 1.2)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->current_action_progress = TAKE_W1_BCo;
            }
            break;
            
        case TAKE_W1_BCo:
            cvs->state->omegaref[R_ID] = -1.5*M_PI;
            cvs->state->omegaref[L_ID] = -1.5*M_PI;
            
            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = BACKWARDS_W1_BCo;
            }
            break;
            
        case BACKWARDS_W1_BCo:
            // TURN ON A*
            cvs->param->gotoPointSpeed = 0;
//            cvs->param->ready_start_astar = 1;
//            cvs->param->Astar_path_active = 0;
//
//            cvs->state->goal_position[0] = 0.0; 
//            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (.8) : (-.85); 
//            cvs->state->goal_position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2); 
            
            cvs->state->intermediate_goal[0] = -0.48; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.7) : (-0.7); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-1.1*M_PI_2) : (1.1*M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // GO TO BRING
            if (((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5)) || cvs->inputs->t - cvs->state->timer > 3) {  
                cvs->state->current_action_progress = BACKWARDS_W2_BCo;
                cvs->state->timer = cvs->inputs->t;
            }
            break;

        case BACKWARDS_W2_BCo:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            cvs->param->gotoPointSpeed = 0;

            cvs->state->intermediate_goal[0] = 0.0; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.1) : (-1.1); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-0.8*M_PI_2) : (0.8*M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // GO TO BRING
            if (((d < 0.07) && (fabs(delta_theta)*180.0/M_PI < 5)) || cvs->inputs->t - cvs->state->timer > 4) {
//            if(!cvs->param->ready_start_astar) {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = BRING_BCo;
            }            
            break;

        case BRING_BCo:
            cvs->param->gotoPointSpeed = 0;
            cvs->state->intermediate_goal[0] = 0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.5) : (-0.5); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-1.2*M_PI_2) : (1.2*M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // GO TO BRING
            if (((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 5)) || (cvs->inputs->t - cvs->state->timer > 4.5)) {
                cvs->state->current_action_progress = UNCLAMP_BCo;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case UNCLAMP_BCo:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -25.0;

            // GO TO MOVE_BACK_END
            if (cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->current_action_progress = MOVE_BACK_BCo;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case MOVE_BACK_BCo:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            cvs->state->intermediate_goal[0] = 0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.8) : (-0.8); 
            cvs->state->intermediate_goal[2] = theta;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if (((d < 0.05) && (delta_theta*180.0/M_PI < 3.5)) || (cvs->inputs->t - cvs->state->timer) > 4)
                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            break;
    }
//    double x, y, theta, x_goal, y_goal, theta_goal;
//    double d, delta_theta;
//    double wheels[2];
//    
//    x = cvs->state->position[0];
//    y = cvs->state->position[1];
//    theta = cvs->state->position[2];
//    x_goal = cvs->state->goal_position[0];
//    y_goal = cvs->state->goal_position[1];
//    theta_goal = cvs->state->goal_position[2];
//
//    switch (cvs->state->current_action_progress) {
//        case GOTO_BD1:
//            // SET GOAL POSITION
//            cvs->state->goal_position[0] = -0.1;
//            cvs->state->goal_position[1] = cvs->inputs->team_color ? -1.1 : 1.1;
//            cvs->state->goal_position[2] = 0;
//
//            // ACTIVATE A*
//            cvs->param->ready_start_astar = 1;
//            cvs->param->Astar_path_active = 0;
//            
//            // CLOSE CLAMP
//            cvs->outputs->command_blocks = 20;
//
//            // GO TO WAIT FOR DESTINATION
//            cvs->state->current_action_progress = WAIT_FOR_POSITION_BD1;
//            break;
//            
//        case WAIT_FOR_POSITION_BD1:
//            // GO TO TURN IF CLOSE ENOUGH
//            if (!cvs->param->ready_start_astar) {
//                cvs->state->objectives[cvs->state->current_objective] = DONE1;
//                cvs->state->current_objective = PARASOL;
//            }
//            break;
//    }
}

void blocks_cabins(CtrlStruct *cvs) {
    double x, y, theta, x_goal, y_goal, theta_goal, d, delta_theta, wheels[2];
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    x_goal = cvs->state->goal_position[0];
    y_goal = cvs->state->goal_position[1];
    theta_goal = cvs->state->goal_position[2];

    switch (cvs->state->current_action_progress) {
        case GOTO_BC:
            // SET GOAL POSITION
            cvs->state->intermediate_goal[0] = -0.5;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.2) : (-1.2);
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2);

            // ACTIVATE A*
//            cvs->param->ready_start_astar = 1;
//            cvs->param->Astar_path_active = 0;

            // GO TO WAIT FOR DESTINATION
            cvs->state->timer = cvs->inputs->t;
            cvs->state->current_action_progress = WAIT_FOR_POSITION_BC;
            break;
            
        case WAIT_FOR_POSITION_BC:
            // COMPUTE REMAINING DISTANCE
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            cvs->param->gotoPointSpeed = 0;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
//            if (!cvs->param->ready_start_astar) {
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3) || cvs->inputs->t - cvs->state->timer > 10)
            // GO TO TURN IF CLOSE ENOUGH
//            if (!cvs->param->ready_start_astar)
            {
                cvs->state->current_action_progress = CALIBRATE_BC;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CALIBRATE_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            
            cvs->state->intermediate_goal[0] = -0.5; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.5) : (-1.5); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = -2.5*M_PI;
            cvs->state->omegaref[L_ID] = -2.5*M_PI;
            
            // GO TO FORWARD 
            if ((!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) || cvs->inputs->t - cvs->state->timer > 1.8)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = POSITION_BC;
                cvs->state->position[1] = (cvs->inputs->team_color) ? (1.36) : (-1.36); 
                cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2); 
            }
            break;
            
        case POSITION_BC:
            cvs->param->gotoPointSpeed = 0;

            cvs->state->intermediate_goal[0] = -0.5; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.57) : (-0.58); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? -M_PI : M_PI; 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            // START OPENING CLAMP
            cvs->outputs->command_blocks = -30;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = 0.7*wheels[R_ID];
            cvs->state->omegaref[L_ID] = 0.7*wheels[L_ID];
            
            if (((d < 0.03) && (fabs(delta_theta)*180.0/M_PI < 2)) || cvs->inputs->t - cvs->state->timer > 5.5) {
                cvs->state->current_action_progress = CALIBRATE_X_BC;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CALIBRATE_X_BC:
            cvs->state->omegaref[R_ID] = -2*M_PI;
            cvs->state->omegaref[L_ID] = -2*M_PI;
            
            // GO TO FORWARD 
            if ((!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID]) || cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = FORWARD_BC;
                cvs->state->position[0] = -0.39; 
                cvs->state->position[2] = -M_PI; 
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
            }
            break;
            
        case FORWARD_BC:
            // TURN OFF A*
            cvs->state->intermediate_goal[0] = -0.79; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.56) : (-0.57); 
            cvs->state->intermediate_goal[2] = M_PI; 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            cvs->param->gotoPointSpeed = 1;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];

            // GO TO CLAMP 
            if ((d < 0.05) || (cvs->inputs->t - cvs->state->timer > 3.0))
            {
                cvs->state->current_action_progress = CLAMP_BC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case CLAMP_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = -0.5*M_PI;
            cvs->state->omegaref[L_ID] = -0.5*M_PI;
            
            cvs->outputs->command_blocks = 75.0;

            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 1.2)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->current_action_progress = TAKE_W1_BC;
            }
            break;
            
        case TAKE_W1_BC:
            cvs->state->omegaref[R_ID] = -1.5*M_PI;
            cvs->state->omegaref[L_ID] = -1.5*M_PI;
            
            // GO TO MOVE_BACK 
            if (cvs->inputs->t - cvs->state->timer > 1)
            {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = BACKWARDS_W1_BC;
            }
            break;
            
        case BACKWARDS_W1_BC:
            // TURN ON A*
            cvs->param->gotoPointSpeed = 1;
//            cvs->param->ready_start_astar = 1;
//            cvs->param->Astar_path_active = 0;
//
//            cvs->state->goal_position[0] = 0.0; 
//            cvs->state->goal_position[1] = (cvs->inputs->team_color) ? (.8) : (-.85); 
//            cvs->state->goal_position[2] = (cvs->inputs->team_color) ? (-M_PI_2) : (M_PI_2); 
            
            cvs->state->intermediate_goal[0] = -0.48; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.7) : (-0.7); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-1.1*M_PI_2) : (1.1*M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // GO TO BRING
            if (((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5)) || cvs->inputs->t - cvs->state->timer > 3) {  
                cvs->state->current_action_progress = BACKWARDS_W2_BC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;

        case BACKWARDS_W2_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;
            cvs->param->gotoPointSpeed = 0;

            cvs->state->intermediate_goal[0] = 0.0; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.1) : (-1.1); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-0.8*M_PI_2) : (0.8*M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // GO TO BRING
            if (((d < 0.07) && (fabs(delta_theta)*180.0/M_PI < 5)) || cvs->inputs->t - cvs->state->timer > 4) {
//            if(!cvs->param->ready_start_astar) {
                cvs->state->timer = cvs->inputs->t;
                cvs->state->current_action_progress = BRING_BC;
            }            
            break;

        case BRING_BC:
            cvs->param->gotoPointSpeed = 0;
            cvs->state->intermediate_goal[0] = 0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.5) : (-0.5); 
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? (-1.2*M_PI_2) : (1.2*M_PI_2); 
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // GO TO BRING
            if (((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 5)) || (cvs->inputs->t - cvs->state->timer > 4.5)) {
                cvs->state->current_action_progress = UNCLAMP_BC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case UNCLAMP_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            // DON'T MOVE
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_blocks = -25.0;

            // GO TO MOVE_BACK_END
            if (cvs->inputs->t - cvs->state->timer > 1.5)
            {
                cvs->state->current_action_progress = MOVE_BACK_BC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case MOVE_BACK_BC:
            // TURN OFF A*
            cvs->param->ready_start_astar = 0;

            cvs->state->intermediate_goal[0] = 0.1; 
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.8) : (-0.8); 
            cvs->state->intermediate_goal[2] = theta;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if (((d < 0.05) && (delta_theta*180.0/M_PI < 3.5)) || (cvs->inputs->t - cvs->state->timer) > 4)
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
//            cvs->state->intermediate_goal[0] = 0.75;
//            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (1.0) : (-0.3);
//            cvs->state->intermediate_goal[2] = M_PI; //(cvs->inputs->team_color) ? -M_PI_2 : M_PI_2;
            cvs->state->intermediate_goal[0] = 0.69;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.9) : (-0.9);
            cvs->state->intermediate_goal[2] = M_PI; //(cvs->inputs->team_color) ? -M_PI_2 : M_PI_2;     
            
            // MAKE SURE CLAMP IS CLOSED
            cvs->outputs->command_blocks = 20;
            // GO TO WAIT FOR DESTINATION
            cvs->state->timer = cvs->inputs->t;
            cvs->state->current_action_progress = WAIT_FOR_POSITION_FC;
            break;
            
        case WAIT_FOR_POSITION_FC:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
//            if (!cvs->param->ready_start_astar) {
            if ((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 2) || cvs->inputs->t - cvs->state->timer > 10) {
                cvs->state->current_action_progress = POSITION_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->param->gotoPointSpeed = 0;
                cvs->state->intermediate_goal[0] = 1.2; //0.75
                cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.83) : (-0.83);
                cvs->state->intermediate_goal[2] = M_PI; //(cvs->inputs->team_color) ? -0.9*M_PI_2 : 0.9*M_PI_2;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case POSITION_FC:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            cvs->param->gotoPointSpeed = 1;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            // ACTION IS DONE
            if ((((!cvs->inputs->u_switch[R_ID] && !cvs->inputs->u_switch[L_ID])) || fabs(cvs->inputs->odo_r_speed - cvs->inputs->r_wheel_speed) > 0.5*M_PI && fabs(cvs->inputs->odo_l_speed - cvs->inputs->l_wheel_speed) > 0.5*M_PI && (cvs->inputs->t - cvs->state->timer > 1.5)) || cvs->inputs->t - cvs->state->timer > 4) {//((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 0.5)) {
                cvs->state->current_action_progress = GO_CATCH_FC;
                cvs->state->position[0] = 0.86;
                cvs->state->position[2] = M_PI;
                cvs->param->gotoPointSpeed = 1;
                cvs->state->intermediate_goal[0] = 0.8;
                cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.9) : (-0.9);
                cvs->state->intermediate_goal[2] = M_PI;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
            }
            break;
            
        case GO_CATCH_FC:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = M_PI; //wheels[R_ID];
            cvs->state->omegaref[L_ID] = M_PI; //wheels[L_ID];
            
            // ACTION IS DONE
            if (cvs->state->position[0] <= 0.85) {
                cvs->state->current_action_progress = PARK_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
        
        case PARK_FC:
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            cvs->param->gotoPointSpeed = 0;
            cvs->state->intermediate_goal[0] = cvs->state->position[0];
            cvs->state->intermediate_goal[1] = cvs->state->position[1];
            cvs->state->intermediate_goal[2] = M_PI_2;
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            // ACTION IS DONE
            if (((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 1) && (cvs->inputs->t - cvs->state->timer > 2.5)) || (cvs->inputs->t - cvs->state->timer > 4)) {
//                if(cvs->inputs->t - cvs->state->timer > 3)
//                    cvs->state->objectives[cvs->state->current_objective] = DELAYED;
                cvs->state->current_action_progress = BRING_OUT_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->timer = cvs->inputs->t;
                brakesMotFH((unsigned int) 0);
                brakesMotFV((unsigned int) 0);
            }
            break;
            
        case BRING_OUT_FC:
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_fish_vertical = -20;
            cvs->outputs->command_fish_horizontal = 40;
            if(cvs->inputs->t - cvs->state->timer > 1.3)
            {
                cvs->state->current_action_progress = LOWER_FC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case LOWER_FC:
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_fish_vertical = 50;
            cvs->outputs->command_fish_horizontal = 30;
            if(cvs->inputs->t - cvs->state->timer > 1.3)
            {
                cvs->outputs->command_fish_vertical = 0.0;
                cvs->outputs->command_fish_horizontal = 0.0;
                cvs->state->current_action_progress = SWEEP_FC;
                cvs->state->timer = cvs->inputs->t;
                cvs->state->errorIntL = 0.0;
                cvs->state->errorIntR = 0.0;
            }
            break;
            
        case SWEEP_FC:
            cvs->state->omegaref[R_ID] = (cvs->inputs->team_color) ? -2*M_PI : 2*M_PI;
            cvs->state->omegaref[L_ID] = (cvs->inputs->team_color) ? -2*M_PI : 2*M_PI;
            if(cvs->inputs->t > cvs->param->tEnd - 3) cvs->state->current_action_progress = GO_UP_FC;
                
            if((fabs(cvs->inputs->odo_r_speed - cvs->inputs->r_wheel_speed) > M_PI && fabs(cvs->inputs->odo_l_speed - cvs->inputs->l_wheel_speed) > M_PI && (cvs->inputs->t - cvs->state->timer > 2)) || cvs->inputs->t - cvs->state->timer > 1.5) {
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->errorIntR = 0.0;
                cvs->state->errorIntL = 0.0;
//                cvs->state->position[0] = .875;
//                cvs->state->position[1] = (cvs->inputs->team_color) ? .715  : (-.71);
//                cvs->state->position[2] = M_PI_2;
//                cvs->outputs->command_fish_vertical = -60;
                cvs->state->current_action_progress = BACKWARDS_FC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case BACKWARDS_FC:
            cvs->state->omegaref[R_ID] = (cvs->inputs->team_color) ? 2*M_PI : -2*M_PI;
            cvs->state->omegaref[L_ID] = (cvs->inputs->team_color) ? 2*M_PI : -2*M_PI;
            if(cvs->inputs->t > cvs->param->tEnd - 3) cvs->state->current_action_progress = GO_UP_FC;

            if(cvs->inputs->t - cvs->state->timer > 1.3) {
                cvs->state->current_action_progress = GO_UP_FC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case GO_UP_FC:
//            cvs->state->omegaref[R_ID] = (cvs->inputs->team_color) ? M_PI : -M_PI;
//            cvs->state->omegaref[L_ID] = (cvs->inputs->team_color) ? M_PI : -M_PI;
            cvs->state->omegaref[R_ID] = 0.0;
            cvs->state->omegaref[L_ID] = 0.0;
            
            cvs->outputs->command_fish_vertical = -60;
            if(cvs->inputs->t - cvs->state->timer > 1.5) 
            {
                cvs->state->current_action_progress = SOMEWHAT_IN_FC;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case SOMEWHAT_IN_FC:
            cvs->outputs->command_fish_horizontal = -25;
            cvs->outputs->command_fish_vertical = -50;
//            
            if(cvs->inputs->t - cvs->state->timer > 1)  
            {
                cvs->state->current_action_progress = LEAVE_SWEEP_FC;
                cvs->state->intermediate_goal[0] = 0.8;
                cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.7) : (-0.7);
                cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? 0.95*M_PI_2 : 0.95*M_PI_2;
                cvs->state->timer = cvs->inputs->t;
//                cvs->state->objectives[cvs->state->current_objective] = DONE1;
            }
            break;
            
        case LEAVE_SWEEP_FC:
            cvs->param->gotoPointSpeed = 0;            
//            cvs->outputs->command_fish_horizontal = -25;
//            cvs->outputs->command_fish_vertical = -50;
            
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            
            if(((d < 0.07) && (fabs(delta_theta)*180.0/M_PI < 5)) || cvs->inputs->t - cvs->state->timer > 6) {
                cvs->state->current_action_progress = GO_NET_FC;
                cvs->state->intermediate_goal[0] = 0.87;
                cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.3) : (-0.3);
                cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? M_PI_2 : M_PI_2;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case GO_NET_FC:            
            cvs->outputs->command_fish_horizontal = -25;
            cvs->outputs->command_fish_vertical = -50;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            cvs->outputs->command_fish_vertical = -50;

            // ACTION IS DONE
            if (((d < 0.05) && (fabs(delta_theta)*180.0/M_PI < 3.5 && cvs->inputs->t - cvs->state->timer > 3))  || cvs->inputs->t - cvs->state->timer > 7) {
                cvs->state->current_action_progress = BRING_IN_FC;
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                cvs->state->timer = cvs->inputs->t;
            }
            break;
            
        case BRING_IN_FC:
            cvs->param->gotoPointSpeed = 0;
            cvs->outputs->command_fish_vertical = -50;
            cvs->outputs->command_fish_horizontal = -75;

            if(cvs->inputs->t - cvs->state->timer > 1.2) {
                cvs->state->current_action_progress = LEAVE_FISH_FC;
                cvs->outputs->command_fish_vertical = 0.0;
                cvs->outputs->command_fish_horizontal = 0.0;
                brakesMotFH((unsigned int) 1);
                brakesMotFV((unsigned int) 1);
            }
            break;
            
        case LEAVE_FISH_FC:
            cvs->state->intermediate_goal[0] = 0.8;
            cvs->state->intermediate_goal[1] = (cvs->inputs->team_color) ? (0.7) : (-0.7);
            cvs->state->intermediate_goal[2] = (cvs->inputs->team_color) ? 0.95*M_PI_2 : 0.95*M_PI_2;
            d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
            delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
            
            gotoPoint(cvs,wheels);
            cvs->state->omegaref[R_ID] = wheels[R_ID];
            cvs->state->omegaref[L_ID] = wheels[L_ID];
            if(((d < 0.07) && (fabs(delta_theta)*180.0/M_PI < 5)) || cvs->inputs->t - cvs->state->timer > 6) {
                                cvs->state->objectives[cvs->state->current_objective] = DONE1;
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
    cvs->outputs->command_fish_vertical = -20;
    cvs->outputs->command_fish_horizontal = 0;
    brakesMotFH((unsigned int) 0);
    brakesMotFV((unsigned int) 1);
    brakesWheels((unsigned int) 1);
    // ACTIVATE PARASOL
    if(cvs->inputs->t > cvs->param->tEnd + 3)
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
    brakesMotFH((unsigned int) 0);
    cvs->outputs->command_fish_vertical = -20;
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