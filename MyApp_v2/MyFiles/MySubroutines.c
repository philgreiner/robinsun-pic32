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
    double x, y, theta, delta_theta, delta_x, delta_y, dist;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];

    double k1, k2, kappa, omega, theta_obj, v, beta, lambda;
    delta_x = destination[0] - x;
    delta_y = destination[1] - y;
    delta_theta = atan2(delta_y,delta_x) - theta;
    delta_theta = (delta_theta > M_PI) ? delta_theta - 2*M_PI : (delta_theta < -M_PI) ? delta_theta + 2*M_PI : delta_theta;
    dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
    
    char msg[1024];
//    sprintf(msg,"delta_theta: %f, dist = %f\n",delta_theta,dist);
//    MyConsole_SendMsg(msg);
    double lin_sat = 0.5, theta_sat = 0.15;
    if(dist > 0.025) // Go to point
    {
        if(-M_PI_2 <= delta_theta && M_PI_2 >= delta_theta) // Go forward
        {
            v = (fabs(delta_theta) > 45.0*M_PI/180.0) ? 0.0 : max(-lin_sat, min(lin_sat, (1.5 * dist)));
            omega = max(-theta_sat, min(theta_sat, (0.25 * delta_theta) - 0.5*(delta_theta-cvs->state->errorAngle)/(cvs->state->lastT - cvs->inputs->t)));
//            sprintf(msg,"Forward ! v: %f, omega: %f\n\n",v,omega);
//            MyConsole_SendMsg(msg);
        }
        else
        {
            delta_theta = (delta_theta < 0) ? delta_theta + M_PI : delta_theta - M_PI;
            v = (fabs(delta_theta) > 45.0*M_PI/180.0) ? 0.0 : -max(-lin_sat, min(lin_sat, (1.5 * dist)));
            omega = max(-theta_sat, min(theta_sat, (0.25 * delta_theta) - 0.5*(delta_theta-cvs->state->errorAngle)/(cvs->state->lastT - cvs->inputs->t)));
//            sprintf(msg,"Backwards ! v: %f, omega: %f\n\n",v,omega);
//            MyConsole_SendMsg(msg);
        }
    }
    else    // Orientation to objective
    {
        delta_theta = destination[2] - theta;
        delta_theta = (delta_theta > 2*M_PI) ? delta_theta - 2*M_PI : (delta_theta < 2*M_PI) ? delta_theta + 2*M_PI : delta_theta;
        v = 0;
        double turn_sat = 0.2;
        omega = max(-turn_sat, min(turn_sat, 0.25 * delta_theta - 0.5*(delta_theta-cvs->state->errorAngle)/(cvs->state->lastT - cvs->inputs->t)));
//        sprintf(msg,"Orientation ! omega: %f\n\n",omega);
//        MyConsole_SendMsg(msg);
    }
    
    wheels[R_ID] = (v + omega)/0.0325;
    wheels[L_ID] = (v - omega)/0.0325;
    cvs->state->errorAngle = delta_theta;
//    theta_obj = -atan2(delta_y,delta_x) + destination[2];
//
//    char msg[1024];
//    k1 = 1.0; k2 = 15.0; beta = 0.4; lambda = 1.0;
//    if(dist > 0.05)
//    {
//        kappa = -1.0/(10.0*dist) * (k2*(delta_theta-atan(-k1*theta_obj)) + (1 + k1 / (1 + powf(k1*theta_obj,2.0)))*sin(delta_theta));
//        v = (0.5/0.0325)/(1 + beta*powf(kappa,lambda));
//        omega = kappa * v;
//        sprintf(msg,"delta_theta = %f, theta_obj = %f, dist = %f, kappa = %f, v = %f, omega = %f;\n",delta_theta, theta_obj, dist,kappa,v,omega);
////        MyConsole_SendMsg(msg);
//    }
//    else
//    {
//        v = 0.0;
//        omega = (destination[2] - theta);
//    }
//    
//    wheels[R_ID] = v + omega;
//    wheels[L_ID] = v - omega;
//
//    double f = (cvs->inputs->t - cvs->state->lastT) / 0.05;
//    double frac = 1.0 / (1.0 + f);
//
//    // filtered_value = { old_value * (dt/tau)/(1+dt/tau) } + { dt/tau * current }
//    wheels[R_ID] = (f * frac * wheels[R_ID]) + (frac * cvs->state->omegaref[R_ID]);
//    wheels[L_ID] = (f * frac * wheels[L_ID]) + (frac * cvs->state->omegaref[L_ID]);
}
#endif