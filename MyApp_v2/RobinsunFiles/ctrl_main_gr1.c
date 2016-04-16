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
        // Right: 5.975 kg     Left: 4.6 kg
        cvs->param->Kp[L_ID] = -0.0631;//-0.1292;
        cvs->param->Ki[L_ID] = 2.1572;//1.2537;
        cvs->param->Kd[L_ID] = 0.001;
        cvs->param->Kp[R_ID] = -0.0548;//-0.1167;
        cvs->param->Ki[R_ID] = 2.3232;//1.6272;
        cvs->param->Kd[R_ID] = 0.001;
    #else
        cvs->param->Kp = -0.031;
        cvs->param->Ki = 2.1729;
    #endif

    // Kalman filter uncertainties
    #ifdef KALMAN // Using Pozyx
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

    cvs->state->position[0] = -0.16;
    cvs->state->position[1] = (cvs->inputs->team_color)? 1.34 : -1.34;
    cvs->state->position[2] = 0;
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

    // Strategy parameters initialization
    cvs->state->objectives_on_robot = 0;
    for (i = 0; i < 7; i = i + 1)
        cvs->state->done_objectives[i] = NOTDONE;

    for (i = 0; i < 10; i++)
        cvs->state->objectives[i] = NOTDONE1;

    cvs->state->strategy_state = WAIT_FOR_START;
    cvs->state->calibration = CALIBRATEY;

    cvs->state->current_objective = CALIBRATE;
    cvs->state->current_action_progress = 0;
    cvs->state->clamp = CLOSED;
    cvs->state->clamp_opening = 9.2;
    cvs->param->refspeed = 0.0;
    cvs->state->errorAngle = 0.0;
    cvs->inputs->u_switch[R_ID] = 1;
    cvs->inputs->u_switch[L_ID] = 1;
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

    cvs->state->clamp_opening += ivs->speed_blocks;
    if (cvs->state->clamp_opening < 10)
        cvs->state->clamp = CLOSED;
    else if (ovs->command_blocks > 5 && ivs->speed_blocks < 1)
        cvs->state->clamp = CLAMPED;
    else if (cvs->state->clamp_opening > 28.5)
        cvs->state->clamp = OPEN;
    else
        cvs->state->clamp = UNCLAMPED;
    char msg[1024];
    sprintf(msg, "Clamp opening: %f; Clamp state: %d\n\n", cvs->state->clamp_opening, (int) cvs->state->clamp);
    //    MyConsole_SendMsg(msg);

    if (fabs(ivs->r_wheel_speed - cvs->state->lastMesR[0]) > 3 * M_PI)
        ivs->r_wheel_speed = cvs->state->lastMesR[0];
    if (fabs(ivs->l_wheel_speed - cvs->state->lastMesL[0]) > 3 * M_PI)
        ivs->l_wheel_speed = cvs->state->lastMesL[0];

    // Computation of the average speed
    int i, j;
    cvs->state->avSpeedR = cvs->state->lastMesR[0];
    cvs->state->avSpeedL = cvs->state->lastMesL[0];
    for (j = 0; j < 6; j++) {
        cvs->state->prevAvSonar[j] = cvs->state->avSonar[j];
        cvs->state->avSonar[j] = cvs->state->lastMesSonar[j][0];
    }

    for (i = 0; i < 9; i++) {
        for (j = 0; j < 6; j++) {
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
        cvs->state->lastMesSonar[i][0] = ivs->sonars[i];

    cvs->state->avSpeedR = cvs->state->avSpeedR / 10.0;
    cvs->state->avSpeedL = cvs->state->avSpeedL / 10.0;
 
    
    for (j = 0; j < 6; j++) {
        //cvs->state->avSonar[j] = ((cvs->state->avSonar[j]/10.0) > 200.0) ? (cvs->state->prevAvSonar[j]) : (cvs->state->avSonar[j]/10.0);
        cvs->state->avSonar[j] = ((cvs->state->avSonar[j]/10.0) < 5.0) ? (cvs->state->prevAvSonar[j]) : (cvs->state->avSonar[j]/10.0);
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

    /* Path planning through potential field computation */
    // Choice of the path planning algorithm
    if (ivs->t >= 5) {
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
                if (cvs->param->Astar_path_active == 0) {
                    Astar_get_path(cvs);
                    cvs->param->Astar_path_active = 1; // A path was found and saved
                }
                Astar_read_path(cvs);
            }
        #endif
    } 
    else {
        cvs->state->omegaref[R_ID] = 0.0;
        cvs->state->omegaref[L_ID] = 0.0;
    }

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

    /* Computation of the motor voltages */
    double wheels[2];
    motors_control(cvs, wheels);
    #ifdef MINIBOT
        ovs->wheel_commands[R_ID] = wheels[L_ID];
        ovs->wheel_commands[L_ID] = wheels[R_ID];
    #else
        ovs->wheel_commands[R_ID] = wheels[R_ID];
        ovs->wheel_commands[L_ID] = wheels[L_ID];
    #endif

    /* Locate the opponent */
    //robot_Detect(cvs);

    if(cvs->state->nb_opponents_detected != 0)
    {
        ovs->wheel_commands[R_ID] = 0.0;
        ovs->wheel_commands[L_ID] = 0.0;
    }
        
    ovs->tower_command = 15;
    cvs->state->lastT = ivs->t;
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
    UconsigneR = (omegaref[R_ID] - rspeed) * cvs->param->Kp[R_ID] + cvs->state->errorIntR * cvs->param->Ki[R_ID] - cvs->param->Kd[R_ID]*(rspeed - cvs->state->lastMesR[0])/(ivs->t - cvs->state->lastT);
    UconsigneL = (omegaref[L_ID] - lspeed) * cvs->param->Kp[L_ID] + cvs->state->errorIntL * cvs->param->Ki[L_ID] - cvs->param->Kd[L_ID]*(lspeed - cvs->state->lastMesL[0])/(ivs->t - cvs->state->lastT);

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