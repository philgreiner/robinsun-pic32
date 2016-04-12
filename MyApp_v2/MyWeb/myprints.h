/* 
 * File:   myprints.h
 * Author: phil_
 *
 * Created on 9 avril 2016, 1:51
 */

#ifndef MYPRINTS_H
#define	MYPRINTS_H

#include "competition_functions.h"

const char* getStateName(robinsun_competition state);
const char* getClampState(clampstate_t state);
const char* getProgress(robinsun_competition state, int current_progress);

#endif	/* MYPRINTS_H */

