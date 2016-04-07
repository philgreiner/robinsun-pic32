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
void controller_init(CtrlStruct *cvs)
{
    int i,j;
    
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
        // Controller parameters
        cvs->param->Kp = 0.0038;
        cvs->param->Ki = 5.5626;

        // Kalman filter uncertainties
        #ifdef KALMAN
            cvs->param->kr = 1;                                 // FIND APPROPRIATE VALUES !
            cvs->param->kl = 1;                                 // FIND APPROPRIATE VALUES !
            for(i=0;i<3;i=i+1)
                for(j=0;j<3;j=j+1)
                    if (i==j)
                    {
                        cvs->state->covariance[i][j] = 1.0;
                        cvs->state->covariance_odo[i][j] = 1.0;
                    }
                    else
                    {
                        cvs->state->covariance[i][j] = 0.0;
                        cvs->state->covariance_odo[i][j] = 0.0;
                    }
        #endif
    #else
        // Controller parameters
        #ifdef ROBINSUN
            cvs->param->Kp = -0.1029; //0.09;
            cvs->param->Ki = 2.0414; //1.18; 
        #else
            cvs->param->Kp = -0.031;
            cvs->param->Ki = 2.1729;
        #endif

        // Kalman filter uncertainties
        #ifdef KALMAN // Using Pozyx
            cvs->param->kr = 1;                                 // FIND APPROPRIATE VALUES !
            cvs->param->kl = 1;                                 // FIND APPROPRIATE VALUES !
            for(i=0;i<3;i=i+1)
                for(j=0;j<3;j=j+1)
                    if (i==j)
                    {
                        cvs->state->covariance[i][j] = 1.0;
                        cvs->state->covariance_odo[i][j] = 1.0;
                    }
                    else
                    {
                        cvs->state->covariance[i][j] = 0.0;
                        cvs->state->covariance_odo[i][j] = 0.0;
                    }
        #endif
    #endif

    cvs->state->lastT = cvs->inputs->t;
  
	/* Sets the beacon position, beginning by the one which is alone on its side, for both robot starting configurations */

    #ifdef SIMU_GAME

        // On which side does the robot begin ? --> Position of the beacons
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

        // If the robot is the blue one
        if(cvs->inputs->robot_id == ROBOT_B)
		{
            cvs->state->position[0] = -0.225;
            cvs->state->position[1] = -1.15;
            cvs->state->position[2] = M_PI_2;
        }
		else if (cvs->inputs->robot_id == ROBOT_R)
		{
			cvs->state->position[0] = -0.075;
			cvs->state->position[1] = -1.4;
			cvs->state->position[2] = M_PI_2;
		}
		else if (cvs->inputs->robot_id == ROBOT_Y)
		{
			cvs->state->position[0] = -0.225;
			cvs->state->position[1] = 1.15;
			cvs->state->position[2] = -M_PI_2;
		}
		else
		{
			cvs->state->position[0] = -0.075;
			cvs->state->position[1] = 1.4;
			cvs->state->position[2] = -M_PI_2;
		}
    #else
        cvs->state->position[0] = -0.225;
        cvs->state->position[1] = -1.15;
        cvs->state->position[2] = M_PI_2;
    #endif

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
        cvs->param->Astar_path_active = 0;          // No path calculated yet
        cvs->param->ready_start_astar = 0;          // No objective inserted yet
    #endif

    // Strategy parameters initialization
    cvs->state->objectives_on_robot = 0;
    for (i=0; i<7; i=i+1)
	   cvs->state->done_objectives[i] = NOTDONE;
    
    for (i=0; i<10; i++)
        cvs->state->objectives[i] = NOTDONE1;

    #ifdef SIMU_PROJECT
        cvs->outputs->flag_release = 0;
    #endif
	cvs->state->strategy_state = WAIT_FOR_START;
	cvs->state->calibration = CALIBRATEY;
    
    cvs->state->current_objective = CALIBRATE;
    cvs->state->current_action_progress = 0;
    cvs->state->clamp = CLOSED;
    cvs->state->clamp_opening = 9.2;
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
    
    cvs->state->clamp_opening += ivs->speed_blocks * (ivs->t - cvs->state->lastT);
    if(cvs->state->clamp_opening < 10)
        cvs->state->clamp = CLOSED;
    else if(ovs->command_blocks > 5 && ivs->speed_blocks < 1)
        cvs->state->clamp = CLAMPED;
    else if(cvs->state->clamp_opening > 28.5)
        cvs->state->clamp = OPEN;
    else
        cvs->state->clamp = UNCLAMPED;
    char msg[1024];
    sprintf("Clamp opening: %f; Clamp state: %d\n", cvs->state->clamp_opening, (int) cvs->state->clamp);
    MyConsole_SendMsg(msg);
    
    if(fabs(ivs->r_wheel_speed - cvs->state->lastMesR[0]) > 3*M_PI)
        ivs->r_wheel_speed = cvs->state->lastMesR[0];
    if(fabs(ivs->l_wheel_speed - cvs->state->lastMesL[0]) > 3*M_PI)
        ivs->l_wheel_speed = cvs->state->lastMesL[0];
    
    // Computation of the average speed
    int i, j;
    cvs->state->avSpeedR = cvs->state->lastMesR[0];
    cvs->state->avSpeedL = cvs->state->lastMesL[0];
    for(j = 0; j<6; j++)
        cvs->state->avSonar[j] = cvs->state->lastMesSonar[j][0];
    
    for(i = 0; i<9; i++)
    {
        for(j = 0; j<6; j++)
        {
            cvs->state->avSonar[j] += cvs->state->lastMesSonar[j][i+1];
            cvs->state->lastMesSonar[j][i+1] = cvs->state->lastMesSonar[j][i];
        }
        cvs->state->avSpeedR += cvs->state->lastMesR[i+1];
        cvs->state->avSpeedL += cvs->state->lastMesL[i+1];
        cvs->state->lastMesR[i+1] = cvs->state->lastMesR[i];
        cvs->state->lastMesL[i+1] = cvs->state->lastMesL[i];
    }
    cvs->state->lastMesR[0] = ivs->r_wheel_speed;
    cvs->state->lastMesL[0] = ivs->l_wheel_speed;
    
    cvs->state->avSpeedR = cvs->state->avSpeedR/10.0;
    cvs->state->avSpeedL = cvs->state->avSpeedL/10.0;
    for(j = 0; j<6; j++)
        cvs->state->avSonar[j] = cvs->state->avSonar[j]/10.0;

    #ifdef SIMU_PROJECT
        set_plot(ivs->r_wheel_speed, "r_wheel_speed");
        set_plot(ivs->l_wheel_speed, "l_wheel_speed");
    #endif

    // Choice of the localization method
    #ifdef KALMAN
	   kalman(cvs);
    #endif
    #ifdef ODO_ONLY
       odometry_estimate(cvs);

       // Debugging
       //set_plot(cvs->state->position_odo[0], "X_odo");
       //set_plot(cvs->state->position_odo[1], "Y_odo");
       //set_plot(cvs->state->position_odo[2], "theta_odo");
    #endif
    #ifdef TRIANG_ONLY
       odometry_estimate(cvs);
       triangulation(cvs);

       // Debugging
       //set_plot(cvs->state->position_odo[0], "X_odo");
       //set_plot(cvs->state->position_odo[1], "Y_odo");
       //set_plot(cvs->state->position_triang[0], "X_triang");
       //set_plot(cvs->state->position_triang[1], "Y_triang");
    #endif

	// Write position errors in separate files (folder ResultsR/ )
	#ifdef SAVE_OUTPUT
        #ifdef KALMAN
    		set_output(cvs->state->position_odo[0], "X_odo_kalman");
    		set_output(cvs->state->position_odo[1], "Y_odo_kalman");
    		set_output(cvs->state->position_odo[2]*180.0/M_PI, "theta_odo_kalman");

            set_output(cvs->state->position_triang[0], "X_triang_kalman");
            set_output(cvs->state->position_triang[1], "Y_triang_kalman");
            set_output(cvs->state->position_triang[2]*180.0/M_PI, "theta_triang_kalman");

            set_output(cvs->state->position[0], "X_kalman");
            set_output(cvs->state->position[1], "Y_kalman");
            set_output(cvs->state->position[2]*180.0/M_PI, "theta_kalman");
        #endif

        #ifdef ODO_ONLY
            set_output(cvs->state->position_odo[0], "X_odo_odo");
            set_output(cvs->state->position_odo[1], "Y_odo_odo");
            set_output(cvs->state->position_odo[2]*180.0/M_PI, "theta_odo_odo");
        #endif

        #ifdef TRIANG_ONLY
            set_output(cvs->state->position_odo[0], "X_odo_triang");
            set_output(cvs->state->position_odo[1], "Y_odo_triang");
            set_output(cvs->state->position_odo[2]*180.0/M_PI, "theta_odo_triang");
            set_output(cvs->state->position_triang[0], "X_triang_triang");
            set_output(cvs->state->position_triang[1], "Y_triang_triang");
            set_output(cvs->state->position_triang[2]*180.0/M_PI, "theta_triang_triang");
        #endif
	#endif

	/* Path planning through potential field computation */
    // Choice of the path planning algorithm
	if (cvs->inputs->t >= 0)
	{
		//strategy_objective(cvs);
        //robinsun_main(cvs);
        
        #ifdef POTENTIAL
            potential_Field(cvs);
        #endif
        #ifdef ASTAR
    		if (cvs->param->ready_start_astar == 1)				// If objective
    		{
    			if (cvs->param->Astar_path_active == 0) {
    				Astar_get_path(cvs);
    				cvs->param->Astar_path_active = 1;			// A path was found and saved
    			}
    			Astar_read_path(cvs);
    		}
        #endif

	}
	else if (cvs->inputs->t < 0)
	{
        #ifdef SIMU_PROJECT
            calibrate_start(cvs);
        #endif
	}
	else
	{
		cvs->state->omegaref[R_ID] = 0.0;
		cvs->state->omegaref[L_ID] = 0.0;
	}

    // Constant speed references for wheel calibrations
    #ifdef TEST_ROBINSUN
        cvs->state->omegaref[R_ID] = (ivs->t > 5)? .30/.0325 : 0;
        cvs->state->omegaref[L_ID] = (ivs->t > 5)? .30/.0325 : 0;
    #endif
    #ifdef TEST_MINIBOT
        cvs->state->omegaref[R_ID] = 4*M_PI*sin(M_PI*ivs->t);
        cvs->state->omegaref[L_ID] = 4*M_PI*cos(M_PI*ivs->t);
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
        if((cvs->state->position_odo[2]> -M_PI_2) && (ivs->t > 5.0)){
            cvs->state->omegaref[R_ID] = -2*M_PI;
            cvs->state->omegaref[L_ID] = 2*M_PI;
        }
        else {
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

    #ifdef ROBINSUN
        //ovs->command_blocks = 100;
    #endif

	/* Locate the opponent */
	//robot_Detect(cvs);

	ovs->tower_command = 15;
	cvs->state->lastT = ivs->t;
    
    #ifdef ROBOTCONSOLE
        char msg[1024];
        sprintf(msg, "x: %.3f; y: %.3f; theta: %.3f\n", cvs->state->position_odo[0], cvs->state->position_odo[1],cvs->state->position_odo[2]*(180.0/M_PI));
        MyConsole_SendMsg(msg);
    #endif
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
void motors_control(CtrlStruct *cvs, double * wheels)
{
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
    
        if (UconsigneR > 0.9 * 24)
        {
            float delta = 0.9 * 24 - UconsigneR;
            UconsigneR = 0.9 * 24;
            UconsigneL += delta;
        }
        if (UconsigneR < -0.9 * 24)
        {
            float delta = -0.9 * 24 - UconsigneR;
            UconsigneR = -0.9 * 24;
            UconsigneL += delta;
        }
        if (UconsigneL > 0.9 * 24)
        {
            float delta = 0.9 * 24 - UconsigneL;
            UconsigneL = 0.9 * 24;
            UconsigneR += delta;
        }
        if (UconsigneL < -0.9 * 24)
        {
            float delta = -0.9 * 24 - UconsigneL;
            UconsigneL = -0.9 * 24;
            UconsigneR += delta;
        }
    
        #ifdef ROBOTCONSOLE
            char msg[1024];
            sprintf(msg, "Right speed: %.3f; omegaref: %.3f; UconsigneR: %.3f, errorIntR = %.3f\n", rspeed, omegaref[R_ID], UconsigneR, cvs->state->errorIntR);
            MyConsole_SendMsg(msg);
            sprintf(msg, "Left speed: %.3f; omegaref: %.3f; UconsigneL: %.3f, errorIntL = %.3f\n", lspeed, omegaref[L_ID], UconsigneL, cvs->state->errorIntL);
            MyConsole_SendMsg(msg);
        #endif

        // Update command values
        #ifdef ROBINSUN
            wheels[R_ID] = -100.0*UconsigneR/26.0;
            wheels[L_ID] = 100.0*UconsigneL/26.0;
            
            double f = (ivs->t - cvs->state->lastT) / 0.05;
            double frac = 1.0 / (1.0 + f);

            // filtered_value = { old_value * (dt/tau)/(1+dt/tau) } + { dt/tau * current }
            wheels[R_ID] = (f * frac * wheels[R_ID]) + (frac * cvs->outputs->wheel_commands[R_ID]);
            wheels[L_ID] = (f * frac * wheels[L_ID]) + (frac * cvs->outputs->wheel_commands[L_ID]);
        #else
            wheels[R_ID] = 100.0*UconsigneR/Valim;
            wheels[L_ID] = 100.0*UconsigneL/Valim;
        #endif
    #endif
}

NAMESPACE_CLOSE();