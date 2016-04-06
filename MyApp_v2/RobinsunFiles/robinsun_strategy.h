/*!
* \file ctrl_strategy_gr1.h
* \brief strategy header of the controller
*/

#ifndef _ROBINSUN_STRATEGY_H_
#define _ROBINSUN_STRATEGY_H_

#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr1);

void robinsun_main(CtrlStruct *cvs);
void robinsun_next(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif