/*!
* \file triangulation_gr1.cc
* \brief File description
*/

#include "namespace_ctrl.h"
#include "CtrlStruct_gr1.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "triangulation_gr1.h"

NAMESPACE_INIT(ctrlGr1);

/*! \brief computation of position via triangulation (called every timestep)
*
* \param[in] inputs structure
* \param[out] outputs written in position[0:2];
*/
void triangulation(CtrlStruct *cvs)
{
    CtrlIn *ivs;
    ivs = cvs->inputs;
	double *beacons = cvs->param->beacons;
	double *position = cvs->state->position;

	/* Algorithm based on power center computation */
	double x1, y1, x2, y2, x3, y3;                                              // Beacon coordinates
	double x1_prime, y1_prime, x3_prime, y3_prime;                              // Modified beacon coordinates wrt to beacon 2
	double T12, T23, T31;                                                       // Cotangent of measured angles
	double x12_prime, y12_prime, x23_prime, y23_prime, x31_prime, y31_prime;    // Modified circle center coordinates
	double k31_prime;                                                           // Constant in power center equation
	double D;                                                                   // Denominator of the final expression
	double xR_iter, yR_iter;
	double prev_dist, current_dist;
	double xR, yR, thetaR;                                                      // Pose of the robot
	double x2_final, y2_final;
	double alpha2_abs;                                                          // Absolute orientation of beacon 2
	double delta_x, delta_y;                                                    // Difference between position of beacon 2 and robot position

	xR = 0.0;
	yR = 0.0;

	/* If rising_index_fixed > falling_index_fixed, the laser is currently ON a beacon.
	the angles that we should look at are therefore the angles before the last rising edge (to look only at 'complete' beacons). */
	int rising_index = ivs->rising_index_fixed;
	int falling_index = ivs->falling_index_fixed;
	if (rising_index > falling_index) rising_index--;

	/* Now look at the three last detected beacons to compute position. */
	double rising[3], falling[3];
	int rising_index_1, rising_index_2;

	if(rising_index == 0){
        rising_index_1 = NB_STORE_EDGE - 1;
        rising_index_2 = NB_STORE_EDGE - 2;
	}
	else if (rising_index == 1){
        rising_index_1 = 0;
        rising_index_2 = NB_STORE_EDGE - 1;
	}
	else {
        rising_index_1 = rising_index - 1;
        rising_index_2 = rising_index - 2;
	}

	rising[0] = ivs->last_rising_fixed[rising_index_2];
	rising[1] = ivs->last_rising_fixed[rising_index_1];
	rising[2] = ivs->last_rising_fixed[rising_index];
	falling[0] = ivs->last_falling_fixed[rising_index_2];
	falling[1] = ivs->last_falling_fixed[rising_index_1];
	falling[2] = ivs->last_falling_fixed[rising_index];

	/* Define the angle of each beacon as the mean between rising and falling edge */
	double alpha[3];
	alpha[0] = (rising[0] + falling[0]) / 2.0;
	alpha[1] = (rising[1] + falling[1]) / 2.0;
	alpha[2] = (rising[2] + falling[2]) / 2.0;

    int i;
	for(i=0;i<3;i=i+1)
    {
        if(i == 0) {
            x1 = beacons[0]; y1 = beacons[1];
            x2 = beacons[2]; y2 = beacons[3];
            x3 = beacons[4]; y3 = beacons[5];
        }
        else if (i == 1) {
            x1 = beacons[2]; y1 = beacons[3];
            x2 = beacons[4]; y2 = beacons[5];
            x3 = beacons[0]; y3 = beacons[1];
        }
        else if (i == 2) {
            x1 = beacons[4]; y1 = beacons[5];
            x2 = beacons[0]; y2 = beacons[1];
            x3 = beacons[2]; y3 = beacons[3];
        }

        /* Step 1 : Compute modified beacon coordinates */
        x1_prime = x1 - x2;
        y1_prime = y1 - y2;
        x3_prime = x3 - x2;
        y3_prime = y3 - y2;

        /* Step 2 : Compute the cotangents */
        T12 = 1.0/tan(alpha[1] - alpha[0]);
        T23 = 1.0/tan(alpha[2] - alpha[1]);
        T31 = (1.0-(T12*T23))/(T12 + T23);

        /* Step 3 : Compute the modified circle center coordinates */
        x12_prime = x1_prime + (T12*y1_prime);
        y12_prime = y1_prime - (T12*x1_prime);
        x23_prime = x3_prime - (T23*y3_prime);
        y23_prime = y3_prime + (T23*x3_prime);
        x31_prime = x3_prime + x1_prime + (T31*(y3_prime - y1_prime));
        y31_prime = y3_prime + y1_prime - (T31*(x3_prime - x1_prime));

        /* Step 4 : Compute k31_prime */
        k31_prime = (x1_prime*x3_prime) + (y1_prime*y3_prime) + T31*((x1_prime*y3_prime)-(x3_prime*y1_prime));

        /* Step 5 : Compute the denominator D */
        D = ((x12_prime-x23_prime)*(y23_prime-y31_prime)) - ((y12_prime-y23_prime)*(x23_prime-x31_prime));

        /* Step 6 : Compute the robot position */
        xR_iter = x2 + ((k31_prime*(y12_prime-y23_prime))/D);
        yR_iter = y2 + ((k31_prime*(x23_prime-x12_prime))/D);

        current_dist = sqrt((xR_iter - position[0])*(xR_iter - position[0]) + (yR_iter - position[1])*(yR_iter - position[1]));
        prev_dist    = sqrt((xR - position[0])*(xR - position[0]) + (yR - position[1])*(yR - position[1]));

        if(i == 0) {
            xR = xR_iter;
            yR = yR_iter;
            x2_final = x2;
            y2_final = y2;
        }
        else if (current_dist < prev_dist) {
            xR = xR_iter;
            yR = yR_iter;
            x2_final = x2;
            y2_final = y2;
        }
    }

    /* Step 7 : Compute the robot orientation, wrt to beacon 2 */
    delta_x = x2_final - xR;
    delta_y = y2_final - yR;

    if((delta_x < 0)&&(delta_y > 0)) { // 2nd quadrant
        alpha2_abs = atan(delta_y/delta_x) + M_PI;
    }
    else if ((delta_x < 0)&&(delta_y < 0)) { // 3rd quadrant
        alpha2_abs = atan(delta_y/delta_x) - M_PI;
    }
    else {
        alpha2_abs = atan(delta_y/delta_x);
    }

    thetaR = alpha[1] - alpha2_abs;
    if (thetaR > M_PI)          thetaR = thetaR - (2.0*M_PI);
    else if (thetaR < -M_PI)    thetaR = thetaR + (2.0*M_PI);

    /* Step 8 : Copy the computed values in the position vector */
    cvs->state->position_triang[0] = xR;
	cvs->state->position_triang[1] = yR;
	cvs->state->position_triang[2] = -thetaR;

}

NAMESPACE_CLOSE();
