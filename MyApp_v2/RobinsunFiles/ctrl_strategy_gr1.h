/*!
* \file ctrl_strategy_gr1.h
* \brief strategy header of the controller
*/

#ifndef _CTRL_STRATEGY_GR1_H_
#define _CTRL_STRATEGY_GR1_H_

#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr1);

void strategy_objective(CtrlStruct *cvs);
void objective_selection(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif