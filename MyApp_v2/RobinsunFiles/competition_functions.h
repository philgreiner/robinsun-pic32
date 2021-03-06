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

/* Functions and enums for all the competition functions */

void calibrate(CtrlStruct *cvs);
void wait(CtrlStruct *cvs);

typedef enum {
    GOTO_BF, WAIT_FOR_POSITION_BF, FORWARD_BF, CLAMP_BF, PUSH_BF, UNCLAMP_BF, MOVE_BACK_BF
} blocks_front_t;
void blocks_front(CtrlStruct *cvs);

typedef enum {GOTO_C, WAIT_FOR_POSITION_C, BACKWARDS_C, SECOND_C_FRONT, SECOND_C, BACKWARDS_C1, SWITCHES_C
} cabins_close_t;
void cabins_close(CtrlStruct *cvs);

typedef enum {GOTO_BD1, WAIT_FOR_POSITION_BD1, CALIBRATE_FIRST_BD1, POSITION_BD1, CALIBRATE_BD1, FORWARD_BD1, CLAMP_BD1, TAKE_W1_BD1, MOVE_BACK_BD1, POS_BD1, BRING_BD1, UNCLAMP_BD1, MOVE_BACK_BD1_END
} blocks_dune_1_t;
void blocks_dune_1(CtrlStruct *cvs);

typedef enum {GOTO_BCo, WAIT_FOR_POSITION_BCo, CALIBRATE_BCo, POSITION_BCo, CALIBRATE_X_BCo, FORWARD_BCo, CLAMP_BCo, TAKE_W1_BCo, BACKWARDS_W1_BCo, BACKWARDS_W2_BCo, BRING_BCo, UNCLAMP_BCo, MOVE_BACK_BCo
} blocks_dune2_t;
void blocks_dune_2(CtrlStruct *cvs);

typedef enum {GOTO_FC, WAIT_FOR_POSITION_FC, POSITION_FC, GO_CATCH_FC, PARK_FC, BRING_OUT_FC, LOWER_FC, SWEEP_FC, BACKWARDS_FC, GO_UP_FC, SOMEWHAT_IN_FC, LEAVE_SWEEP_FC, GO_NET_FC, BRING_IN_FC, LEAVE_FISH_FC, GO_CATCH_FC2, PARK_FC2, BRING_OUT_FC2, LOWER_FC2, SWEEP_FC2, BACKWARDS_FC2, GO_UP_FC2, SOMEWHAT_IN_FC2, LEAVE_SWEEP_FC2, GO_NET_FC2, BRING_IN_FC2, LEAVE_FISH_FC2
} fish_catch_t;
void fish_catch(CtrlStruct *cvs);

typedef enum {GOTO_BC, WAIT_FOR_POSITION_BC, CALIBRATE_BC, POSITION_BC, CALIBRATE_X_BC, FORWARD_BC, CLAMP_BC, TAKE_W1_BC, BACKWARDS_W1_BC, BACKWARDS_W2_BC, BRING_BC, UNCLAMP_BC, MOVE_BACK_BC
} blocks_cabins_t;
void blocks_cabins(CtrlStruct *cvs);
void parasol_open(CtrlStruct *cvs);
void stop_end(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif