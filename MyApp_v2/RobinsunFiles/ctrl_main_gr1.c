/*!
 * \file ctrl_main_gr1.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

// Basic includes
#include "ctrl_main_gr1.h"
#include "MyApp.h"

//#define ROBOTCONSOLE

NAMESPACE_INIT(ctrlGr1);

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs) {
    int i, j;

    /* Parameters of the wheel speed control */
    cvs->state->errorIntR = 0.0;
    cvs->state->errorIntL = 0.0;
    cvs->state->avSpeedR = 0.0;
    cvs->state->avSpeedL = 0.0;
    cvs->inputs->mode = 0;

    int zeta;
    for (zeta = 0; zeta < 10; zeta++) {
        cvs->state->lastMesL[zeta] = 0;
        cvs->state->lastMesR[zeta] = 0;
    }
    
    for (zeta = 0; zeta < 10; zeta++)
        for(i = 0; i < 6; i++)
            cvs->state->lastMesSonar[i][zeta] = 0.0;

    // Controller parameters
    #ifdef ROBINSUN
        //                  Right: 5.975 kg     Left: 4.6 kg
        cvs->param->Kp[L_ID] = 0.1487;
        cvs->param->Ki[L_ID] = 2.0111;
        cvs->param->Kd[L_ID] = 0.008;
        cvs->param->Kp[R_ID] = 0.1381;
        cvs->param->Ki[R_ID] = 2.2223;
        cvs->param->Kd[R_ID] = 0.001;
    #else
        cvs->param->Kp = -0.031;
        cvs->param->Ki = 2.1729;
    #endif

    cvs->param->kphi = 0.003962;
    cvs->param->linsatv = 0.75;
    cvs->param->linsatw = 1.5;
    cvs->param->angsatv = 0.45;
    cvs->param->angsatw = 0.45;
    cvs->param->kpangv = 10;
    cvs->param->kpangw = 0.8;
    cvs->param->kdangv = 5;
    cvs->param->kdangw = 0.275;
    cvs->param->kplin = 5;
    cvs->param->distmin = 0.7;
    cvs->param->minspeed = 0.5;
    cvs->param->anglev = 25.0;
    cvs->param->refangle = 0;
    cvs->param->xref = -0.16;
    cvs->param->yref = -1.34;
    // Kalman filter uncertainties
    #ifdef KALMAN // Using Decawave
        cvs->param->kr = 1; // FIND APPROPRIATE VALUES !
        cvs->param->kl = 1; // FIND APPROPRIATE VALUES !
        for (i = 0; i < 3; i = i + 1)
            for (j = 0; j < 3; j = j + 1)
                if (i == j) {
                    cvs->state->covariance[i][j] = 1.0;
                    cvs->state->covariance_odo[i][j] = 1.0;
                } else {
                    cvs->state->covariance[i][j] = 0.0;
                    cvs->state->covariance_odo[i][j] = 0.0;
                }
    #endif

    cvs->state->lastT = cvs->inputs->t;

    cvs->state->position[0] = (cvs->inputs->team_color) ? (-0.09) : (-0.16);
    cvs->state->position[1] = (cvs->inputs->team_color) ? (1.34) : (-1.34);
    cvs->state->position[2] = (cvs->inputs->team_color) ? (-M_PI) : (0);
    cvs->state->prev_theta = 0;

    cvs->state->position_odo[0] = cvs->state->position[0];
    cvs->state->position_odo[1] = cvs->state->position[1];
    cvs->state->position_odo[2] = cvs->state->position[2];
    cvs->state->position_triang[0] = cvs->state->position[0];
    cvs->state->position_triang[1] = cvs->state->position[1];
    cvs->state->position_triang[2] = cvs->state->position[2];
    cvs->state->direction = 0;

    // Path planning strategy selection
    #ifdef POTENTIAL
        potential_Field_Init(cvs);
    #endif
    #ifdef ASTAR
        game_map_drawing(cvs);
        cvs->param->Astar_path_active = 0; // No path calculated yet
        cvs->param->ready_start_astar = 0; // No objective inserted yet
    #endif

    for (i = 0; i < 10; i++)
        cvs->state->objectives[i] = NOTDONE1;

    for(i=0;i<3;i++) {
        cvs->state->intermediate_goal[i] = 0.0;
        cvs->state->goal_position[i] = 0.0;
    }
    cvs->state->current_objective = CALIBRATE;
    cvs->state->current_action_progress = 0;
    cvs->param->refspeed = 0.0;
    cvs->state->errorAngle = 0.0;
    cvs->state->errorDist = 0.0;
    cvs->inputs->u_switch[R_ID] = 1;
    cvs->inputs->u_switch[L_ID] = 1;
    cvs->state->opponent_timer = -42.0;
    
    // Initializing last valid Astar position
    cvs->state->last_astarPos[0] = 0.0;
    cvs->state->last_astarPos[1] = (cvs->inputs->team_color) ? (1.0) : (-1.0);
    
    cvs->param->gotoPointSpeed = 0;
    
    cvs->state->i_save = 0;
    cvs->param->tEnd = 120.0;
}

/*! \brief controller loop (called eveiry timestep)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs) {
    CtrlIn *ivs;
    CtrlOut *ovs;

    ivs = cvs->inputs;
    ovs = cvs->outputs;

    if (fabs(ivs->r_wheel_speed)*0.0325 > .9)
    {
        ivs->r_wheel_speed = cvs->state->lastMesR[0];
    }
    if (fabs(ivs->l_wheel_speed)*0.0325 > .9)
    {
        ivs->l_wheel_speed = cvs->state->lastMesL[0];
    }

    // Computation of the average speed
    int i, j;
    int count[6] = {0};
    double sumyes[6] = {0.0};
    cvs->state->avSpeedR = cvs->state->lastMesR[0];
    cvs->state->avSpeedL = cvs->state->lastMesL[0];
    for (j = 0; j < 6; j++) {
        cvs->state->prevAvSonar[j] = cvs->state->avSonar[j];
        cvs->state->avSonar[j] = cvs->state->lastMesSonar[j][0];
        if(cvs->state->lastMesSonar[j][0] < 90)
        {
            sumyes[j] += cvs->state->lastMesSonar[j][0];
            count[j]++;
        }
    }

    for (i = 0; i < 9; i++) {
        for (j = 0; j < 6; j++) {
            if(cvs->state->lastMesSonar[j][i+1] < 90)
            {
                sumyes[j] += cvs->state->lastMesSonar[j][i + 1];
                count[j]++;
            }
            cvs->state->avSonar[j] += cvs->state->lastMesSonar[j][i + 1];
            cvs->state->lastMesSonar[j][i + 1] = cvs->state->lastMesSonar[j][i];
        }
        cvs->state->avSpeedR += cvs->state->lastMesR[i + 1];
        cvs->state->avSpeedL += cvs->state->lastMesL[i + 1];
        cvs->state->lastMesR[i + 1] = cvs->state->lastMesR[i];
        cvs->state->lastMesL[i + 1] = cvs->state->lastMesL[i];
    }
    cvs->state->lastMesR[0] = ivs->r_wheel_speed;
    cvs->state->lastMesL[0] = ivs->l_wheel_speed;
    for(i = 0; i < 6; i++)
    {
        if(count[j] >= 5)
            cvs->state->avSonar[j] = sumyes[j]/count[j];
        else
            cvs->state->avSonar[j] = 42000;
        cvs->state->lastMesSonar[i][0] = ivs->sonars[i];
    }

    cvs->state->avSpeedR = cvs->state->avSpeedR / 10.0;
    cvs->state->avSpeedL = cvs->state->avSpeedL / 10.0;
    if(fabs(ivs->r_wheel_speed - cvs->state->avSpeedR) > 6*M_PI)
    {
        ivs->r_wheel_speed = cvs->state->lastMesR[1];
    }
    if(fabs(ivs->l_wheel_speed - cvs->state->avSpeedL) > 6*M_PI)
    {
        ivs->l_wheel_speed = cvs->state->lastMesL[1];
    }
    
    // Retrieve sonar values
    for (j = 0; j < 6; j++) {
        //cvs->state->avSonar[j] = ((cvs->state->avSonar[j]/10.0) > 200.0) ? (cvs->state->prevAvSonar[j]) : (cvs->state->avSonar[j]/10.0);
        cvs->state->avSonar[j] = ((cvs->state->avSonar[j]/10.0) < 5.0) ? (cvs->state->prevAvSonar[j]) : (cvs->state->avSonar[j]/10.0);
    }
    
    // Save last valid Astar position
    int x_astar = (int) (cvs->state->position[0] * 20 + 21);                      
	int y_astar = (int) (cvs->state->position[1] * 20 + 31);
    
    if(cvs->param->game_map[x_astar][y_astar] && cvs->state->position[0] > -0.7) {
        cvs->state->last_astarPos[0] = (double) (x_astar - 21.0)/20.0;
        cvs->state->last_astarPos[1] = (double) (y_astar - 31.0)/20.0;
    }

    // Choice of the localization method
    #ifdef KALMAN
        kalman(cvs);
    #endif
    #ifdef ODO_ONLY
        odometry_estimate(cvs);
    #endif
    #ifdef TRIANG_ONLY
        odometry_estimate(cvs);
        triangulation(cvs);
    #endif

        if(cvs->param->refspeed == 0.0)
        {
        /* Locate the opponent */
        locate_opponent(cvs);

            /* Path planning through potential field computation */
            // Choice of the path planning algorithm
                //strategy_objective(cvs);
        
        robinsun_main(cvs);
    //        cvs->state->omegaref[R_ID] = cvs->param->refspeed/.0325;
    //        cvs->state->omegaref[L_ID] = cvs->param->refspeed/.0325;
        #ifdef POTENTIAL
            potential_Field(cvs);
        #endif
        #ifdef ASTAR
            if (cvs->param->ready_start_astar == 1) // If objective
            {
                if (!cvs->param->Astar_path_active)
                    Astar_get_path(cvs);
                if(cvs->param->Astar_path_active)
                    Astar_read_path(cvs);
            }
        #endif

        // Constant speed references for wheel calibrations
        #ifdef TEST_ROBINSUN
            cvs->state->omegaref[R_ID] = (ivs->t > 5) ? .30 / .0325 : 0;
            cvs->state->omegaref[L_ID] = (ivs->t > 5) ? .30 / .0325 : 0;
        #endif
        #ifdef TEST_MINIBOT
            cvs->state->omegaref[R_ID] = 4 * M_PI * sin(M_PI * ivs->t);
            cvs->state->omegaref[L_ID] = 4 * M_PI * cos(M_PI * ivs->t);
        #endif

        // Tests to calibrate the odometry
        //#define TEST_ODO
        #ifdef TEST_ODO
            //        if((cvs->state->position_odo[1]<1.85) && (ivs->t > 5.0)){
            //            cvs->state->omegaref[R_ID] = 3*M_PI;
            //            cvs->state->omegaref[L_ID] = 3*M_PI;
            //        }
            //        else {
            //            cvs->state->omegaref[R_ID] = 0;
            //            cvs->state->omegaref[L_ID] = 0;
            //        }
            if ((cvs->state->position_odo[2]> -M_PI_2) && (ivs->t > 5.0)) {
                cvs->state->omegaref[R_ID] = -2 * M_PI;
                cvs->state->omegaref[L_ID] = 2 * M_PI;
            } else {
                cvs->state->omegaref[R_ID] = 0;
                cvs->state->omegaref[L_ID] = 0;
            }
        #endif

    //    if(cvs->inputs->start_signal) {
    //            cvs->state->intermediate_goal[0] = cvs->param->xref; 
    //            cvs->state->intermediate_goal[1] = cvs->param->yref; 
    //            cvs->state->intermediate_goal[2] = cvs->param->refangle*M_PI/180.0;
    //            double x = cvs->state->position[0], y = cvs->state->position[1]; 
    //            double wheels[2], d = sqrt((x - cvs->state->intermediate_goal[0])*(x - cvs->state->intermediate_goal[0]) + (y - cvs->state->intermediate_goal[1])*(y - cvs->state->intermediate_goal[1]));
    //            double delta_theta = fabs(cvs->state->position[2] - cvs->state->intermediate_goal[2]);
    //            delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : delta_theta;
    //            
    //            gotoPoint(cvs,wheels);
    //            cvs->state->omegaref[R_ID] = wheels[R_ID];
    //            cvs->state->omegaref[L_ID] = wheels[L_ID];
    //    }
        }
        else
        {
            if(cvs->param->refspeed == 1.0 || cvs->param->refspeed == 2.0)
            {
                cvs->state->intermediate_goal[0] = cvs->param->xref; 
                cvs->state->intermediate_goal[1] = cvs->param->yref; 
            }
            else 
            {
                cvs->state->intermediate_goal[0] = cvs->state->position[0]; 
                cvs->state->intermediate_goal[1] = cvs->state->position[1]; 
            }
            if(cvs->param->refspeed == 1.0 || cvs->param->refspeed == 3.0)
                cvs->state->intermediate_goal[2] = cvs->param->refangle*M_PI/180.0;
            else
                cvs->state->intermediate_goal[2] = cvs->state->position[2];
                double wheels[2];
                gotoPoint(cvs,wheels);
                cvs->state->omegaref[R_ID] = wheels[R_ID];
                cvs->state->omegaref[L_ID] = wheels[L_ID];
        }

    /* Computation of the motor voltages */
    //cvs->state->omegaref[R_ID] = M_PI;
    //cvs->state->omegaref[L_ID] = M_PI;
    
//        if(cvs->inputs->start_signal)
//        {
//            cvs->state->omegaref[R_ID] = (ivs->t > 5 && ivs->t < 7.5) ? .30 / .0325 : 0;
//            cvs->state->omegaref[L_ID] = (ivs->t > 5 && ivs->t < 7.5) ? .30 / .0325 : 0;
////            char msg[1024];
////            sprintf(msg, "%.3f %.3f %.3f %.3f\n", cvs->state->omegaref[L_ID], cvs->inputs->l_wheel_speed, cvs->state->omegaref[R_ID], cvs->inputs->r_wheel_speed);
////            MyConsole_SendMsg(msg);
//        }
        
    double wheels[2];
    if(ivs->t != 0.0)
    {
        motors_control(cvs, wheels);
    #ifdef MINIBOT
        ovs->wheel_commands[R_ID] = wheels[L_ID];
        ovs->wheel_commands[L_ID] = wheels[R_ID];
    #else
        ovs->wheel_commands[R_ID] = wheels[R_ID];
        ovs->wheel_commands[L_ID] = wheels[L_ID];
    #endif
    }

//    if(cvs->state->nb_opponents_detected != 0)
//    {
//        ovs->wheel_commands[R_ID] = 0.0;
//        ovs->wheel_commands[L_ID] = 0.0;
//    }
        
    cvs->state->lastT = ivs->t;
    
    // Save data during the game
    if(cvs->inputs->start_signal && cvs->state->i_save < 250)
    {
        cvs->state->mes_speed[R_ID][cvs->state->i_save] = ivs->r_wheel_speed;
        cvs->state->mes_speed[L_ID][cvs->state->i_save] = ivs->l_wheel_speed;
        cvs->state->ref_speed[R_ID][cvs->state->i_save] = cvs->state->omegaref[R_ID];
        cvs->state->ref_speed[L_ID][cvs->state->i_save] = cvs->state->omegaref[L_ID];
        cvs->state->theTime[cvs->state->i_save] = cvs->inputs->t;
//        sprintf(&(cvs->state->mes_speed[R_ID][0]),"%s %.3f",&(cvs->state->mes_speed[R_ID][0]),ivs->r_wheel_speed);
//        sprintf(&(cvs->state->mes_speed[R_ID][0]),"%s %.3f",&(cvs->state->mes_speed[L_ID][0]),ivs->r_wheel_speed);
//        sprintf(&(cvs->state->mes_speed[R_ID][0]),"%s %.3f",&(cvs->state->ref_speed[R_ID][0]),cvs->state->omegaref[R_ID]);
//        sprintf(&(cvs->state->mes_speed[R_ID][0]),"%s %.3f",&(cvs->state->ref_speed[L_ID][0]),cvs->state->omegaref[L_ID]);

//        cvs->state->est_pos[0][cvs->state->i_save] = cvs->state->position[0];
//        cvs->state->est_pos[1][cvs->state->i_save] = cvs->state->position[1];
//        cvs->state->est_pos[2][cvs->state->i_save] = cvs->state->position[2];
        MyConsole_SendMsg("Saving data\n");
        cvs->state->i_save++;
    }
    else
//        MyConsole_SendMsg("Waiting for start to save\n");
    
    // Save to SD card at the end
    if(cvs->state->i_save > 249 && cvs->state->i_save != 4001)
    {
        cvs->state->i_save = 4001;
        char msg[1024];
        sprintf(msg,"time = %.3f", cvs->inputs->t);
        MyConsole_SendMsg(msg);
        MyDataSave(cvs);
        sprintf(msg,"time = %.3f", cvs->inputs->t);
        MyConsole_SendMsg(msg);
    }
}

/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs) {

}

/*! \brief motor controllers (called every timestep)
 *
 * \param[in] position, reference_pos : current position and reference position
 * \param[out] outputs written in wheels[0:1];
 */
void motors_control(CtrlStruct *cvs, double * wheels) {
    double UconsigneR, UconsigneL;
    double Valim = 24.0;
    CtrlIn *ivs = cvs->inputs;
    double *omegaref = cvs->state->omegaref;

    // Get values for current speed and reference speed
    #ifdef ROBINSUN
        double rspeed = ivs->r_wheel_speed;
        double lspeed = ivs->l_wheel_speed;
    #else
        #ifdef MINIBOT
            double rspeed = -ivs->r_wheel_speed;
            double lspeed = -ivs->l_wheel_speed;
        #else
            double rspeed = ivs->r_wheel_speed;
            double lspeed = ivs->l_wheel_speed;
        #endif
    #endif

    // Integrate the error
    cvs->state->errorIntR += (omegaref[R_ID] - rspeed)*(ivs->t - cvs->state->lastT);
    cvs->state->errorIntL += (omegaref[L_ID] - lspeed)*(ivs->t - cvs->state->lastT);

    // Limit the integral error (anti-windup)
    cvs->state->errorIntR = (cvs->state->errorIntR * cvs->param->Ki[R_ID] > Valim) ? (Valim / (cvs->param->Ki[R_ID])) : (cvs->state->errorIntR);
    cvs->state->errorIntL = (cvs->state->errorIntL * cvs->param->Ki[L_ID] > Valim) ? (Valim / (cvs->param->Ki[L_ID])) : (cvs->state->errorIntL);
    cvs->state->errorIntR = (cvs->state->errorIntR * cvs->param->Ki[R_ID]<-Valim) ? (-Valim / (cvs->param->Ki[R_ID])) : (cvs->state->errorIntR);
    cvs->state->errorIntL = (cvs->state->errorIntL * cvs->param->Ki[L_ID]<-Valim) ? (-Valim / (cvs->param->Ki[L_ID])) : (cvs->state->errorIntL);

    // PI controller
    UconsigneR = (omegaref[R_ID] - rspeed) * cvs->param->Kp[R_ID] + cvs->state->errorIntR * cvs->param->Ki[R_ID] - cvs->param->Kd[R_ID]*(rspeed - cvs->state->lastMesR[0])/(ivs->t - cvs->state->lastT) + cvs->param->kphi*rspeed;
    UconsigneL = (omegaref[L_ID] - lspeed) * cvs->param->Kp[L_ID] + cvs->state->errorIntL * cvs->param->Ki[L_ID] - cvs->param->Kd[L_ID]*(lspeed - cvs->state->lastMesL[0])/(ivs->t - cvs->state->lastT) + cvs->param->kphi*lspeed;

    if (UconsigneR > 0.9 * 24) {
        float delta = 0.9 * 24 - UconsigneR;
        UconsigneR = 0.9 * 24;
        UconsigneL += delta;
    }
    if (UconsigneR < -0.9 * 24) {
        float delta = -0.9 * 24 - UconsigneR;
        UconsigneR = -0.9 * 24;
        UconsigneL += delta;
    }
    if (UconsigneL > 0.9 * 24) {
        float delta = 0.9 * 24 - UconsigneL;
        UconsigneL = 0.9 * 24;
        UconsigneR += delta;
    }
    if (UconsigneL < -0.9 * 24) {
        float delta = -0.9 * 24 - UconsigneL;
        UconsigneL = -0.9 * 24;
        UconsigneR += delta;
    }

    // Update command values
    #ifdef ROBINSUN
        wheels[R_ID] = -100.0 * UconsigneR / 26.0;
        wheels[L_ID] = 100.0 * UconsigneL / 26.0;

        double f = (ivs->t - cvs->state->lastT) / 0.005;
        double frac = 1.0 / (1.0 + f);

        // filtered_value = { old_value * (dt/tau)/(1+dt/tau) } + { dt/tau * current }
        wheels[R_ID] = (f * frac * wheels[R_ID]) + (frac * cvs->outputs->wheel_commands[R_ID]);
        wheels[L_ID] = (f * frac * wheels[L_ID]) + (frac * cvs->outputs->wheel_commands[L_ID]);
    #else
        wheels[R_ID] = 100.0 * UconsigneR / Valim;
        wheels[L_ID] = 100.0 * UconsigneL / Valim;
    #endif
}

NAMESPACE_CLOSE();