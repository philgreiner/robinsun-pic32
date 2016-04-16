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

void gotoPoint(CtrlStruct *cvs, double *destination, double *wheels)
{
    double x, y, theta, delta_theta, delta_x, delta_y, dist;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];

    double v, omega;
    delta_x = destination[0] - x;
    delta_y = destination[1] - y;
    delta_theta = atan2(delta_y,delta_x) - theta;
    delta_theta = (delta_theta > M_PI) ? delta_theta - 2*M_PI : (delta_theta < -M_PI) ? delta_theta + 2*M_PI : delta_theta;
    dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
    
    char msg[1024];
    
    double lin_sat = 0.4, theta_sat = 0.2;
    if(dist > 0.035) // Go to point
    {
        if(-M_PI_2 <= delta_theta && M_PI_2 >= delta_theta) // Go forward
        {
            v = (fabs(delta_theta) > 45.0*M_PI/180.0) ? 0.0 : max(-lin_sat, min(lin_sat, (1.5 * dist)));
            omega = max(-theta_sat, min(theta_sat, (0.225 * delta_theta) + 0.5*(delta_theta - cvs->state->errorAngle)/(cvs->inputs->t - cvs->state->lastT)));
        }
        else
        {
            delta_theta = (delta_theta < 0) ? delta_theta + M_PI : delta_theta - M_PI;
            v = (fabs(delta_theta) > 45.0*M_PI/180.0) ? 0.0 : -max(-lin_sat, min(lin_sat, (1.5 * dist)));
            omega = max(-theta_sat, min(theta_sat, (0.225 * delta_theta) + 0.5*(delta_theta - cvs->state->errorAngle)/(cvs->inputs->t - cvs->state->lastT)));
        }
    }
    else    // Orientation to objective
    {
        delta_theta = destination[2] - theta;
        delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : (delta_theta < -M_PI) ? (delta_theta + 2*M_PI) : delta_theta;
        v = 0;
        omega = 0.125 * delta_theta;
    }
    
    wheels[R_ID] = (v + omega)/0.0325;
    wheels[L_ID] = (v - omega)/0.0325;
    cvs->state->errorAngle = delta_theta;
    cvs->state->prev_theta = theta;
}
#endif