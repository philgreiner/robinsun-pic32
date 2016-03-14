/*! 
 * \file ododmetry_gr1.h
 * \brief File description
 */
#ifndef _ODOMETRY_GR1_H_ 
#define _ODOMETRY_GR1_H_

#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr1); 

void odometry_estimate(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif // end of header guard