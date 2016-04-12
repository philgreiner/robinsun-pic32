/*!
 * \file robot_detect_gr1.h
 * \brief Locate the adversary robot
 */
#ifndef _ROBOT_DETECT_GR1_H_ // adapt it with the name of this file (header guard)
#define _ROBOT_DETECT_GR1_H_ // must be the same name as the line before

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h" // adapt it with your headers
#include "triangulation_gr1.h"	// For the lowpass function

#define TAU_DETECT 0.75

NAMESPACE_INIT(ctrlGr1);

void robot_Detect(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif