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
 	
 	// Step 1
 	double alpha_pred[3];
    double r_pred[3];

    // Step 2
 	double H[6][3];
 	double H_transposed[3][6];

 	// Step 3
 	double v[6][3];
 	double measure_covariance[6][6];
 	double innovation_covariance[6][6];
 	double inverse_innovation_covariance[6][6];

 	// Step 4
 	double Kt[3][6];
 	double Kt_transposed[6][3];

} kalmanStruct;

void kalman(CtrlStruct *cvs);
void measurement_update(CtrlStruct *cvs);
void acquire_triang_measurements(CtrlStruct * cvs);
void measure_covariance_compute(CtrlStruct * cvs, kalmanStruct * kal);
void innovation_covariance_compute(CtrlStruct * cvs, kalmanStruct * kal);
void kalman_gain_compute(CtrlStruct * cvs, kalmanStruct * kal);
void inverse(kalmanStruct * kal);

NAMESPACE_CLOSE();

#endif
