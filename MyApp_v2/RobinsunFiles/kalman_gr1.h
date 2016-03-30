/*!
 * \file kalman_gr1.h
 * \brief main header of the controller
 */

#ifndef _KALMAN_GR1_H_
#define _KALMAN_GR1_H_

#include "CtrlStruct_gr1.h"
#include "odometry_gr1.h"
#include "triangulation_gr1.h"
#include <stdio.h>

 NAMESPACE_INIT(ctrlGr1); 

typedef struct kalmanStruct{
	/* This structure makes the code easier to implement */

    // Step 2
 	double H[3][3];

 	// Step 3
 	double v[3];
 	double measure_covariance[3][3];
 	double innovation_covariance[3][3];
 	double inverse_innovation_covariance[3][3];

 	// Step 4
 	double Kt[3][3];

} kalmanStruct;

void kalman(CtrlStruct *cvs);
void innovation_covariance_compute(CtrlStruct * cvs, kalmanStruct * kal);
void kalman_gain_compute(CtrlStruct * cvs, kalmanStruct * kal);
void inverse(double matrix [3][3], kalmanStruct * kal);
void update_position(CtrlStruct * cvs, kalmanStruct * kal);
void mult_matrix33_31(double A[3][3], double B[3], double result[3]);

NAMESPACE_CLOSE();

#endif