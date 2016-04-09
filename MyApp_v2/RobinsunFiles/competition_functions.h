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

typedef enum {
    GOTO_BF, WAIT_FOR_POSITION_BF, TURN_BF, FORWARD_BF, CLAMP_BF, PUSH_BF, UNCLAMP_BF, MOVE_BACK_BF
} blocks_front_t;
void blocks_front(CtrlStruct *cvs);

typedef enum {GOTO_C, WAIT_FOR_POSITION_C, TURN_C, BACKWARDS_C, SWITCHES_C
} cabins_close_t;
void cabins_close(CtrlStruct *cvs);

typedef enum {GOTO_BD1, WAIT_FOR_POSITION_BD1, TURN_BD1, FORWARD_BD1, CLAMP_BD1, MOVE_BACK_BD1, BRING_BD1, UNCLAMP_BD1, MOVE_BACK_BD1_END
} blocks_dune_1_t;
void blocks_dune_1(CtrlStruct *cvs);
void blocks_dune_2(CtrlStruct *cvs);

typedef enum {GOTO_BC, WAIT_FOR_POSITION_BC, TURN_BC, FORWARD_BC, CLAMP_BC, BACKWARDS_BC, BRING_BC, UNCLAMP_BC, MOVE_BACK_BC
} blocks_cabins_t;
void blocks_cabins(CtrlStruct *cvs);
void fish_catch(CtrlStruct *cvs);
void parasol_open(CtrlStruct *cvs);
void stop_end(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif