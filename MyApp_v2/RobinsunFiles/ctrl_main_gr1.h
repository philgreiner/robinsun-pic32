/*!
 * \file ctrl_main_gr1.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR1_H_
#define _CTRL_MAIN_GR1_H_

#ifdef SIMU_PROJECT
    #define SIMU_GAME // comment this line to see in simulation the controller which will be tested on the real robot
    #define SAVE_OUTPUT
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "CtrlStruct_gr1.h"
#include "namespace_ctrl.h"
 
// Simulation and results
#ifdef SIMU_PROJECT
    #include "set_output.h"
    #include "user_realtime.h"
    #include "robot_id.h"
#endif

// Localization mode
#include "odometry_gr1.h"

#ifdef KALMAN
    #include "kalman_gr1.h"
    #include "triangulation_gr1.h"
#endif
#ifdef TRIANG_ONLY
    #include "triangulation_gr1.h"
#endif

// Path planning strategy
#ifdef POTENTIAL
    #include "potentialfield_gr1.h"
#endif
#ifdef ASTAR
    #include "Astar_gr1.h"
    #include "map_design_gr1.h"
#endif

#include "robot_detect_gr1.h"
#include "ctrl_strategy_gr1.h"
#include "calibrate_gr1.h"

NAMESPACE_INIT(ctrlGr1);

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);
void motors_control(CtrlStruct *cvs, double *wheels);

NAMESPACE_CLOSE();

#endif
