/*!
 * \file CtrlStruct_gr1.h
 * \brief Controller main structure
 */

#ifndef _LOCATE_OPPONENT_GR1_H_
#define _LOCATE_OPPONENT_GR1_H_

#include "ctrl_io.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "namespace_ctrl.h"
#include "ctrl_main_gr1.h"

NAMESPACE_INIT(ctrlGr1);

void locate_opponent(CtrlStruct *cvs);
int isMeasureRelevant(CtrlStruct *cvs, double xFound, double yFound);

NAMESPACE_CLOSE();

#endif
