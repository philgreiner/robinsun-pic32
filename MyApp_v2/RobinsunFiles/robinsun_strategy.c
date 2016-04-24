#include "robinsun_strategy.h"
#include <math.h>
#ifdef SIMU_PROJECT
#include "robot_id.h"
#endif

NAMESPACE_INIT(ctrlGr1);

void robinsun_main(CtrlStruct *cvs) {
    if(cvs->inputs->t >= 90 && cvs->inputs->start_signal && cvs->state->objectives[PARASOL] != DONE1)
        cvs->state->current_objective = PARASOL;
    switch (cvs->state->current_objective) {
        case CALIBRATE:
            calibrate(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case WAIT_FOR_START1:
            wait(cvs);

            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_FRONT:
            cvs->state->this_objective_timer = 9;
            blocks_front(cvs);
//            if(cvs->inputs->t - cvs->state->objective_timer > cvs->state->this_objective_timer)
//                cvs->state->objectives[BLOCKS_FRONT] = DONE1;
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case CABINS:
            cvs->state->this_objective_timer = 15;
            cabins_close(cvs);
//            if(cvs->inputs->t - cvs->state->objective_timer > cvs->state->this_objective_timer)
//                cvs->state->objectives[CABINS] = DONE1;
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_DUNE_1:
            cvs->state->this_objective_timer = 25;
            blocks_dune_1(cvs);
//            if(cvs->inputs->t - cvs->state->objective_timer > cvs->state->this_objective_timer)
//                cvs->state->objectives[BLOCKS_DUNE_1] = DONE1;
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_DUNE_2:
            blocks_dune_2(cvs);
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case BLOCKS_CABINS:
            cvs->state->this_objective_timer = 25;
            blocks_cabins(cvs);
//            if(cvs->inputs->t - cvs->state->objective_timer > cvs->state->this_objective_timer)
//                cvs->state->objectives[BLOCKS_CABINS] = DONE1;
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case FISH:
            cvs->state->this_objective_timer = 25;
            fish_catch(cvs);
            
//            if(cvs->inputs->t - cvs->state->objective_timer > cvs->state->this_objective_timer)
//                cvs->state->objectives[FISH] = DONE1;
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1 | cvs->state->objectives[cvs->state->current_objective] == DELAYED)
                robinsun_next(cvs);
            break;

        case PARASOL:
            parasol_open(cvs);
            
            if (cvs->state->objectives[cvs->state->current_objective] == DONE1)
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
                cvs->state->current_objective = (robinsun_competition) i;
                cvs->state->objective_timer = cvs->inputs->t;
                return;
            }
        
        // IF ALL ARE DONE, TRY TO DO THE OBJECTIVES THAT WERE INTERRUPTED
        for(i = 0; i < 8; i++)
            if(cvs->state->objectives[i] == DELAYED) {
                cvs->state->objectives[i] = NOTDONE1;
                cvs->state->current_objective = (robinsun_competition) i;
                cvs->state->objective_timer = cvs->inputs->t;
                return;
            }
        
        cvs->state->current_objective = PARASOL;
        // IF EVERYTHING IS DONE, TAKE MORE BLOCKS
//        cvs->state->objectives[BLOCKS_DUNE_2] = NOTDONE1;
//        cvs->state->current_objective = BLOCKS_DUNE_2;
//        cvs->state->objective_timer = cvs->inputs->t;
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