#include "robinsun_strategy.h"
#include <math.h>
#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

void robinsun_main(CtrlStruct *cvs) {
    switch (cvs->state->current_objective) {
        case CALIBRATE:
            calibrate(cvs);
            MyConsole_SendMsg("In calibration mode.\n");

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case WAIT_FOR_START1:
            wait(cvs);
            MyConsole_SendMsg("In waiting mode.\n");

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_FRONT:
            blocks_front(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case CABINS:
            cabins_close(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_DUNE_1:
            blocks_dune_1(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_DUNE_2:
            blocks_dune_2(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_CABINS:
            blocks_cabins(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case FISH:
            fish_catch(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case PARASOL:
            parasol_open(cvs);
            cvs->state->current_objective = STOP;
            break;

        case STOP:
            stop_end(cvs);
            break;
    }
}

void robinsun_next(CtrlStruct *cvs) {
    cvs->state->current_action_progress = 0;
    if(cvs->inputs->t > 89)
        cvs->state->current_objective = PARASOL;
    else {
        int i; 
        // FIRST DO THE NOTDONE OBJECTIVES
        for(i = 0; i < 8; i++)
            if(cvs->state->objectives[i] == NOTDONE1) {
                char msg[1024];
                sprintf(msg, "i = %d\n", i);
                MyConsole_SendMsg(msg);
                cvs->state->current_objective == (robinsun_competition) i;
                return;
            }
        
        // IF ALL ARE DONE, TRY TO DO THE OBJECTIVES THAT WERE INTERRUPTED
        for(i = 0; i < 8; i++)
            if(cvs->state->objectives[i] == DELAYED) {
                cvs->state->current_objective == (robinsun_competition) i;
                return;
            }
        
        // IF EVERYTHING IS DONE, TAKE MORE BLOCKS
        cvs->state->objectives[BLOCKS_DUNE_2] = NOTDONE;
        cvs->state->current_objective = BLOCKS_DUNE_2;
    }
}

NAMESPACE_CLOSE();

/*
________________________________________________________________________________________________________________________________________________________
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                  ||                                                                         ||                                       |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                              _____________________________________________________                                                   |
|____________                                 |_________________________  __________________________|                                      ____________|
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|  PURPLE    |                                           BASE           | |        BASE                                                   |   GREEN    |
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|            |                                                          | |                                                               |            |
|____________|                                                          |_|                                                               |____________|
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|                                                                                                                                                      |
|__                                                                                                                                                  __|
|  \__                                                                                                                                            __/  |
|     \__                                                                                                                                      __/     |
|________\____________________________________________________________________________________________________________________________________/________|

 */