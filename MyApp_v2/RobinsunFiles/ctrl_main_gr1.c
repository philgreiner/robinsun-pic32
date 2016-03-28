/*!
 * \file ctrl_main_gr1.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef SIMU_PROJECT
// Simulation and results
    #include "set_output.h"
    #include "user_realtime.h"
    #include "robot_id.h"
#endif
#define MANUAL

// User defined files
#include "potentialfield_gr1.h"
#include "odometry_gr1.h"
#include "Astar_gr1.h"
#include "map_design_gr1.h"
#include "triangulation_gr1.h"
#include "robot_detect_gr1.h"
#include "kalman_gr1.h"
#include "ctrl_strategy_gr1.h"

NAMESPACE_INIT(ctrlGr1);

#ifdef SIMU_PROJECT
    #define SIMU_GAME // comment this line to see in simulation the controller which will be tested on the real robot
#endif

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
    /* Parameters of the wheel speed control */
	cvs->state->errorIntR = 0.0;
	cvs->state->errorIntL = 0.0;
    cvs->state->avSpeedR = 0.0;
    cvs->state->avSpeedL = 0.0;
    
    int zeta;
    for(zeta=0;zeta<10;zeta++)
    {
        cvs->state->lastMesL[zeta] = 0;
        cvs->state->lastMesR[zeta] = 0;
    }
    
    #ifdef SIMU_GAME
        cvs->param->Kp = 0.1043;
        cvs->param->Ki = 28.98;
    #else
        #ifdef ROBINSUN
            cvs->param->Kp = 0.09;
            cvs->param->Ki = 1.18; 
        #else
            cvs->param->Kp = -0.0426;
            cvs->param->Ki = 1.3907;
        #endif
    #endif

    cvs->state->lastT = cvs->inputs->t;

	/* Parameters and state variables used in the Kalman filter */
	int i,j;
	for(i=0;i<3;i=i+1)
		for(j=0;j<3;j=j+1)
			cvs->state->covariance[i][j] = 0.0;

	cvs->param->kr = 1;									// FIND APPROPRIATE VALUES !
	cvs->param->kl = 1;									// FIND APPROPRIATE VALUES !
	cvs->param->R_beacon = 0.04;

	/* Sets the beacon position, beginning by the one which is alone on its side, for both robot starting configurations */
    #ifdef SIMU_GAME
        if((cvs->inputs->robot_id == ROBOT_B)||(cvs->inputs->robot_id == ROBOT_R)){
            cvs->param->beacons[0] = 0;         cvs->param->beacons[1] = -1.562;
            cvs->param->beacons[2] = 1.062;     cvs->param->beacons[3] = 1.562;
            cvs->param->beacons[4] = -1.062;    cvs->param->beacons[5] = 1.562;
        }
        else {
            cvs->param->beacons[0] = 0;         cvs->param->beacons[1] = 1.562;
            cvs->param->beacons[2] = -1.062;    cvs->param->beacons[3] = -1.562;
            cvs->param->beacons[4] = 1.062;     cvs->param->beacons[5] = -1.562;
        }

        if(cvs->inputs->robot_id == ROBOT_B){
            cvs->state->position[0] = -0.225;
            cvs->state->position[1] = -1.15;
            cvs->state->position[2] = M_PI_2;
            cvs->state->position_odo[0] = -0.225;
            cvs->state->position_odo[1] = -1.15;
            cvs->state->position_odo[2] = M_PI_2;
        }
    #else 
        cvs->state->position[0] = -0.225;
        cvs->state->position[1] = -1.15;
        cvs->state->position[2] = M_PI_2;
        cvs->state->position_odo[0] = -0.225;
        cvs->state->position_odo[1] = -1.15;
        cvs->state->position_odo[2] = M_PI_2;
    #endif

	potential_Field_Init(cvs);

	cvs->state->done_objectives[0] = NOTDONE; 
    cvs->state->done_objectives[1] = NOTDONE; 
    cvs->state->done_objectives[2] = NOTDONE; 
    cvs->state->done_objectives[3] = NOTDONE;
	cvs->state->done_objectives[4] = NOTDONE; 
    cvs->state->done_objectives[5] = NOTDONE; 
    cvs->state->done_objectives[6] = NOTDONE;
	cvs->state->objectives_on_robot = 0;
    
    #ifdef SIMU_PROJECT
        cvs->outputs->flag_release = 0;
    #endif
	cvs->state->strategy_state = WAIT_FOR_START;
}

/*! \brief controller loop (called eveiry timestep)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
	CtrlIn *ivs;
	CtrlOut *ovs;

	ivs = cvs->inputs;
	ovs = cvs->outputs;

//    if(fabs(ivs->r_wheel_speed - cvs->state->lastMesR[0]) > 7)
//        ivs->r_wheel_speed = cvs->state->avSpeedR;
//    if(fabs(ivs->r_wheel_speed - cvs->state->lastMesR[0]) > 7)
//        ivs->l_wheel_speed = cvs->state->avSpeedL;
    
    int i;
    for(i = 0; i<9; i++)
    {
        cvs->state->lastMesR[i+1] = cvs->state->lastMesR[i];
        cvs->state->lastMesL[i+1] = cvs->state->lastMesL[i];
    }
    cvs->state->lastMesR[0] = ivs->r_wheel_speed;
    cvs->state->lastMesL[0] = ivs->l_wheel_speed;
    
    cvs->state->avSpeedR = 0.0;
    cvs->state->avSpeedL = 0.0;
    
    for(i = 0; i<10; i++)
    {
        cvs->state->avSpeedR += cvs->state->lastMesR[i];
        cvs->state->avSpeedL += cvs->state->lastMesL[i];
    }
    
    cvs->state->avSpeedR = cvs->state->avSpeedR/10.0;
    cvs->state->avSpeedL = cvs->state->avSpeedL/10.0;
    
    #ifdef SIMU_PROJECT
        set_plot(ivs->r_wheel_speed, "r_wheel_speed");
        set_plot(ivs->l_wheel_speed, "l_wheel_speed");
    #endif

	odometry_estimate(cvs);
    cvs->state->position[2] = cvs->state->position_odo[2];
	cvs->state->position[0] = cvs->state->position_odo[0];
	cvs->state->position[1] = cvs->state->position_odo[1];

	#ifdef SIMU_PROJECT
		//printf("Odometry x:%f [m] ; y:%f [m] ; angle:%f [deg]\n", cvs->state->position_odo[0], cvs->state->position_odo[1], cvs->state->position_odo[2]*180.0/M_PI);
        triangulation(cvs);
        kalman(cvs);	// includes odometry and triangulation
	#endif
	
	#ifdef SAVE_OUTPUT
		set_output(cvs->state->position_odo[0], "X_odo");
		set_output(cvs->state->position_odo[1], "Y_odo");
		set_output(cvs->state->position_odo[2]*180.0/M_PI, "theta_odo");
	#endif

	/* Path planning through potential field computation */

	strategy_objective(cvs);
    potential_Field(cvs);

    #ifdef ROBINSUN
        cvs->state->omegaref[R_ID] = (ivs->t > 5)? .30/.0325 : 0;
        cvs->state->omegaref[L_ID] = (ivs->t > 5)? .30/.0325 : 0;
    #endif
    
    #ifdef MINIBOT
        cvs->state->omegaref[R_ID] = 0.2/0.03;
        cvs->state->omegaref[L_ID] = 0.2/0.03;
    #endif
    
    
	/* Computation of the motor voltages */
	double wheels[2];
	motors_control(cvs, cvs->state->position_odo, wheels);
    
    #ifdef MINIBOT
        ovs->wheel_commands[R_ID] = wheels[L_ID];
        ovs->wheel_commands[L_ID] = wheels[R_ID];
    #else
        ovs->wheel_commands[R_ID] = wheels[R_ID];
        ovs->wheel_commands[L_ID] = wheels[L_ID];
    #endif

    #ifdef ROBINSUN
        ovs->command_blocks = 50;
    #endif

	/* Locate the opponent */
	//robot_Detect(cvs);

    #ifdef DEBUG
        printf("x0 = %.3f, y0 = %.3f, x1 = %.3f, y1 = %.3f \n", cvs->state->opponent_position[0], cvs->state->opponent_position[1],cvs->state->opponent_position[2],cvs->state->opponent_position[3]);
    #endif

	ovs->tower_command = 15;
	cvs->state->lastT = ivs->t;
}

/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}

/*! \brief motor controllers (called every timestep)
*
* \param[in] position, reference_pos : current position and reference position
* \param[out] outputs written in wheels[0:1];
*/
void motors_control(CtrlStruct *cvs, double * position, double * wheels)
{
    double UconsigneR, UconsigneL;
    double Valim = 24.0;
	CtrlIn *ivs = cvs->inputs;
	double *omegaref = cvs->state->omegaref;

	// Get values for current speed and reference speed
    #ifdef ROBINSUN
        double rspeed = cvs->state->avSpeedR;
        double lspeed = cvs->state->avSpeedL;
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
    
     #ifdef ROBOTCONSOLE
        char msg[1024];
        sprintf(msg, "t: %.3f; lastT: %.3f \n", ivs->t, cvs->state->lastT);
        MyConsole_SendMsg(msg);
    #endif

#ifdef SIMU_PROJECT
	// Limit the integral error (anti-windup)
	cvs->state->errorIntR = (14 * cvs->state->errorIntR*cvs->param->Ki>24.0) ? (24.0 / (14 * cvs->param->Ki)) : (cvs->state->errorIntR);
	cvs->state->errorIntL = (14 * cvs->state->errorIntL*cvs->param->Ki>24.0) ? (24.0 / (14 * cvs->param->Ki)) : (cvs->state->errorIntL);
	cvs->state->errorIntR = (14 * cvs->state->errorIntR*cvs->param->Ki<-24.0) ? (-24.0 / (14 * cvs->param->Ki)) : (cvs->state->errorIntR);
	cvs->state->errorIntL = (14 * cvs->state->errorIntL*cvs->param->Ki<-24.0) ? (-24.0 / (14 * cvs->param->Ki)) : (cvs->state->errorIntL);

	// PI controller
	UconsigneR = (omegaref[R_ID] - rspeed) * 14 * cvs->param->Kp + cvs->state->errorIntR * 14 * cvs->param->Ki;
	UconsigneL = (omegaref[L_ID] - lspeed) * 14 * cvs->param->Kp + cvs->state->errorIntL * 14 * cvs->param->Ki;

	UconsigneR = (UconsigneR>0.9*24) ? (0.9*24) : (UconsigneR);
	UconsigneL = (UconsigneL>0.9*24) ? (0.9*24) : (UconsigneL);

	wheels[R_ID] = UconsigneR*(100/(0.9*24));
	wheels[L_ID] = UconsigneL*(100/(0.9*24));
#else
    // Limit the integral error (anti-windup)
    cvs->state->errorIntR = (cvs->state->errorIntR*cvs->param->Ki>Valim) ? ( Valim/(cvs->param->Ki)) : (cvs->state->errorIntR);
    cvs->state->errorIntL = (cvs->state->errorIntL*cvs->param->Ki>Valim) ? ( Valim/(cvs->param->Ki)) : (cvs->state->errorIntL);
    cvs->state->errorIntR = (cvs->state->errorIntR*cvs->param->Ki<-Valim)? (-Valim/(cvs->param->Ki)) : (cvs->state->errorIntR);
    cvs->state->errorIntL = (cvs->state->errorIntL*cvs->param->Ki<-Valim)? (-Valim/(cvs->param->Ki)) : (cvs->state->errorIntL);

    // PI controller
    UconsigneR = (omegaref[R_ID]-rspeed)*cvs->param->Kp + cvs->state->errorIntR*cvs->param->Ki;
    UconsigneL = (omegaref[L_ID]-lspeed)*cvs->param->Kp + cvs->state->errorIntL*cvs->param->Ki;

    UconsigneR = (UconsigneR>Valim) ? ( Valim) : (UconsigneR);
    UconsigneR = (UconsigneR<-Valim)? (-Valim) : (UconsigneR);
    UconsigneL = (UconsigneL>Valim) ? ( Valim) : (UconsigneL);
    UconsigneL = (UconsigneL<-Valim)? (-Valim) : (UconsigneL);
    
    #ifdef ROBOTCONSOLE
        sprintf(msg, "Right speed: %.3f; omegaref: %.3f; UconsigneR: %.3f, errorIntR = %.3f\n", rspeed, omegaref[R_ID], UconsigneR, cvs->state->errorIntR);
        MyConsole_SendMsg(msg);
        sprintf(msg, "Left speed: %.3f; omegaref: %.3f; UconsigneL: %.3f, errorIntL = %.3f\n", lspeed, omegaref[L_ID], UconsigneL, UconsigneR, cvs->state->errorIntL);
        MyConsole_SendMsg(msg);
    #endif

    // Update command values
    #ifdef ROBINSUN
        wheels[R_ID] = -100.0*UconsigneR/26.0;
        wheels[L_ID] = 100.0*UconsigneL/26.0;
    #else
        wheels[R_ID] = 100.0*UconsigneR/Valim;
        wheels[L_ID] = 100.0*UconsigneL/Valim;
    #endif
#endif
}

NAMESPACE_CLOSE();
