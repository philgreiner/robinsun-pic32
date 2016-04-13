/*! 
 * \author Philippe Greiner
 * \file MySubroutines.c
 * \brief Functions to easily send instructions to motors 
 *          and simplify the robot control.
 */

#ifndef MySubroutines
#define MySubroutines

#include "MyApp.h"
#include <math.h>

// duty between -100 and 100
void    DutyToInf(double duty, unsigned *MSB, unsigned *LSB)
{
    int realduty = (int) 128 + 128.0*duty/100.0;

    *MSB = (int) realduty/4;
    if(realduty%2 == 0)
        if(realduty%4 == 0)
            *LSB = 0;
        else
            *LSB = 2;
    else
        if((realduty-1)%4 == 0)
            *LSB = 1;
        else
            *LSB = 3;
}

void    gotoPoint(CtrlStruct *cvs, double *destination, double *wheels)
{
    double x, y, theta, delta_theta, delta_x, delta_y;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    double dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));

    delta_x = destination[0] - x;
    delta_y = destination[1] - y;
    delta_theta = atan2(delta_y,delta_x) - theta;
    if(dist < 0.001) delta_theta = destination[2] - theta;
    if (delta_theta > M_PI) delta_theta -= 2 * M_PI;
    if (delta_theta < -M_PI) delta_theta += 2 * M_PI;
        
    double omega = 3*delta_theta;
    omega = (omega > 3*M_PI) ? (3*M_PI) : omega;
    omega = (omega < -3*M_PI) ? (-3*M_PI) : omega;

    double vlin = 1.0*M_PI*cos(delta_theta);
    if (fabs(delta_theta) < 0.7*M_PI_4) {
        vlin = (vlin > 2.75*M_PI) ? (2.75*M_PI) : (vlin);
        vlin = (vlin < -2.75*M_PI) ? (-2.75*M_PI) : (vlin);
        if (fabs(vlin) <= M_PI_4)
            vlin = (vlin /(fabs(vlin)))*M_PI_4;
    }
    else {
        vlin = 0;
    }

    wheels[R_ID] = vlin + omega;
    wheels[L_ID] = vlin - omega;
}
#endif