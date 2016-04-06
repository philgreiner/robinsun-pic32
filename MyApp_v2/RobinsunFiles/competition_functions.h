/*!
 * \file ctrl_strategy_gr1.h
 * \brief strategy header of the controller
 */

#ifndef _COMPETITION_FUNCTIONS_H_
#define _COMPETITION_FUNCTIONS_H_

#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr1);

void calibrate(CtrlStruct *cvs);
void wait(CtrlStruct *cvs);
void blocks_front(CtrlStruct *cvs);
void cabins_close(CtrlStruct *cvs);
void blocks_dune_1(CtrlStruct *cvs);
void blocks_dune_2(CtrlStruct *cvs);
void blocks_cabins(CtrlStruct *cvs);
void fish_catch(CtrlStruct *cvs);
void parasol_open(CtrlStruct *cvs);
void stop_end(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif