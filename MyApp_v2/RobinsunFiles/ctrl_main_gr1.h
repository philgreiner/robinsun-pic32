/*!
 * \file ctrl_main_gr1.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR1_H_
#define _CTRL_MAIN_GR1_H_

#include "CtrlStruct_gr1.h"
#include "namespace_ctrl.h"
#include "odometry_gr1.h"
#include "map_design_gr1.h"
#include "potentialfield_gr1.h"
#include "triangulation_gr1.h"
#include "robot_detect_gr1.h"
#include <stdlib.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr1);

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);
void motors_control(CtrlStruct *cvs, double *position, double *wheels);

NAMESPACE_CLOSE();

#endif
