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
    GOTO_BF, WAIT_FOR_POSITION_BF, FORWARD_BF, CLAMP_BF, PUSH_BF, UNCLAMP_BF, MOVE_BACK_BF
} blocks_front_t;
void blocks_front(CtrlStruct *cvs);

typedef enum {GOTO_C, WAIT_FOR_POSITION_C, BACKWARDS_C, SECOND_C, BACKWARDS_C1, SWITCHES_C
} cabins_close_t;
void cabins_close(CtrlStruct *cvs);

typedef enum {GOTO_BD1, WAIT_FOR_POSITION_BD1, CALIBRATE_BD1, FORWARD_BD1, CLAMP_BD1, MOVE_BACK_BD1, POS_BD1, BRING_BD1, UNCLAMP_BD1, MOVE_BACK_BD1_END
} blocks_dune_1_t;
void blocks_dune_1(CtrlStruct *cvs);
void blocks_dune_2(CtrlStruct *cvs);

typedef enum {GOTO_FC, WAIT_FOR_POSITION_FC, POSITION_FC, GO_CATCH_FC, PARK_FC, BRING_OUT_FC, LOWER_FC, SWEEP_FC, BACKWARDS_FC, GO_UP_FC, SOMEWHAT_IN_FC, LEAVE_SWEEP_FC, GO_NET_FC, BRING_IN_FC
} fish_catch_t;
void fish_catch(CtrlStruct *cvs);

typedef enum {GOTO_BC, WAIT_FOR_POSITION_BC, CALIBRATE_BC, POSITION_BC, FORWARD_BC, CLAMP_BC, BACKWARDS_W1_BC, BACKWARDS_W2_BC, BRING_BC, UNCLAMP_BC, MOVE_BACK_BC
} blocks_cabins_t;
void blocks_cabins(CtrlStruct *cvs);
void parasol_open(CtrlStruct *cvs);
void stop_end(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif