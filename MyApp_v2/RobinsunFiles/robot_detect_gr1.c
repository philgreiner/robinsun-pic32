/*!
* \file robot_detect_gr1.cc
* \brief File description
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "namespace_ctrl.h"
#include "ctrl_main_gr1.h"

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

void robot_Detect(CtrlStruct *cvs)
{
    CtrlIn *ivs;
    ivs = cvs->inputs;
    int n_robots = ivs->nb_opponents;
    double* position = cvs->state->position_odo;
    double robot_pos[4];
    int rising_index[2];
    int falling_index;

    /* If rising_index > falling_index the laser is currently ON a beacon. */

    #ifdef DEBUG
        printf("n_robots = %d \n", n_robots);
    #endif

    /* If rising_index_fixed > falling_index_fixed, the laser is currently ON a beacon.
	the angles that we should look at are therefore the angles before the last rising edge (to look only at 'complete' beacons). */
	rising_index[0] = ivs->rising_index;
	falling_index = ivs->falling_index;
	if (rising_index[0] > falling_index) rising_index[0] = rising_index[0] - 1;

    /* Now look at the three last detected beacons to compute position. */
	double rising[2], falling[2];
	int rising_index_1;

	if(rising_index[0] == 0){
        rising_index_1 = NB_STORE_EDGE - 1;
	}
	else {
        rising_index_1 = rising_index[0] - 1;
	}

	if (n_robots > 1)   rising_index[1] = rising_index_1;

    int i;
	for(i=0; i<n_robots; i++)
    {
        rising[i] = ivs->last_rising[rising_index[i]];
        falling[i] = ivs->last_falling[rising_index[i]];
    }


    int index = 0;
    for(i=0; i< n_robots; i++)
    {
        /* Calculation of the orientation */
        double angle = position[2] + ((rising[i] + falling[i])/2.0);
        if(angle > M_PI) {
            angle = angle - (2*M_PI);
        }
        else if(angle < -M_PI) {
            angle = angle + (2*M_PI);
        }

        /* Calculation of the distance */
        double R = 0.1; // Radius of the beacon
        double d = R/tan(falling[i] - rising[i]);


        /* Calculation of the opponent position */
        robot_pos[index] = position[0] + d*cos(angle);
        robot_pos[index + 1] = position[1] + d*sin(angle);

        if ((robot_pos[index] < 1.0) && (robot_pos[index] > -1.0))    cvs->state->opponent_position[index] = robot_pos[index];
        if ((robot_pos[index + 1] < 1.5) && (robot_pos[index + 1] > -1.5))    cvs->state->opponent_position[index + 1] = robot_pos[index + 1];
        index += 2;
    }
}

NAMESPACE_CLOSE();
