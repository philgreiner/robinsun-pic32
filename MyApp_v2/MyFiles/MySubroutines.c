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

    double k1, k2, kappa, omega, theta_obj, v, beta, lambda;
    delta_x = destination[0] - x;
    delta_y = destination[1] - y;
    delta_theta = -atan2(delta_y,delta_x) + theta;
    theta_obj = -atan2(delta_y,delta_x) + destination[2];
    double dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));

    char msg[1024];
    k1 = 1.0; k2 = 3.0; beta = 0.4; lambda = 2.0;
    if(dist > 0.05)
    {
        kappa = -1/dist * (k2*(delta_theta-atan(-k1*theta_obj)) + (1 + k1 / (1 + powf(k1*theta_obj,2.0)))*sin(delta_theta));
        v = (0.3/0.0325)/(1 + beta*powf(kappa,lambda));
        omega = kappa * v/4.0;
        sprintf(msg,"delta_theta = %f, theta_obj = %f, dist = %f, kappa = %f, v = %f, omega = %f;\n",delta_theta, theta_obj, dist,kappa,v,omega);
//        MyConsole_SendMsg(msg);
    }
    else
    {
        v = 0.0;
        omega = destination[2] - theta;
    }
    
    //    if(dist < 0.001) delta_theta = destination[2] - theta;
//    if (delta_theta > M_PI) delta_theta -= 2 * M_PI;
//    if (delta_theta < -M_PI) delta_theta += 2 * M_PI;
//        
//    double omega = 3*delta_theta;
//    omega = (omega > 3*M_PI) ? (3*M_PI) : omega;
//    omega = (omega < -3*M_PI) ? (-3*M_PI) : omega;
//
//    double vlin = 1.0*M_PI*cos(delta_theta);
//    if (fabs(delta_theta) < 0.7*M_PI_4) {
//        vlin = (vlin > 2.75*M_PI) ? (2.75*M_PI) : (vlin);
//        vlin = (vlin < -2.75*M_PI) ? (-2.75*M_PI) : (vlin);
//        if (fabs(vlin) <= M_PI_4)
//            vlin = (vlin /(fabs(vlin)))*M_PI_4;
//    }
//    else {
//        vlin = 0;
//    }

    wheels[L_ID] = v - omega;
    wheels[R_ID] = v + omega;
}
#endif