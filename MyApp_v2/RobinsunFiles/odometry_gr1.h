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
void mult_matrix32_22(double A[3][2], double B[2][2], double result[3][2]);
void mult_matrix32_23(double A[3][2], double B[2][3], double result[3][3]);
void mult_matrix33_33(double A[3][3], double B[3][3], double result[3][3]);

NAMESPACE_CLOSE();

#endif // end of header guard