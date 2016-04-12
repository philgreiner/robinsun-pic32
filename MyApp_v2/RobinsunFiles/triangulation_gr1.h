/*!
 * \file triangulation_gr1.h
 * \brief Triangulation
 */
#ifndef _TRIANGULATION_GR1_H_ // adapt it with the name of this file (header guard)
#define _TRIANGULATION_GR1_H_ // must be the same name as the line before

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef SIMU_PROJECT
    #include "set_output.h"
    #include "user_realtime.h"
    #include "robot_id.h"
#endif

#define TAU_LOWPASS 0.5

NAMESPACE_INIT(ctrlGr1);

void triangulation(CtrlStruct *cvs);
double lowpass(double old, double current, double tau, CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif