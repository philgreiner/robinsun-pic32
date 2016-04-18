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

void gotoPoint(CtrlStruct *cvs, double *wheels)
{
    double destination[3], x, y, theta, delta_theta, delta_x, delta_y, dist;
    x = cvs->state->position[0];
    y = cvs->state->position[1];
    theta = cvs->state->position[2];
    destination[0] = cvs->state->intermediate_goal[0];
    destination[1] = cvs->state->intermediate_goal[1];
    destination[2] = cvs->state->intermediate_goal[2];
    
    double v, omega;
    delta_x = destination[0] - x;
    delta_y = destination[1] - y;
    delta_theta = atan2(delta_y,delta_x) - theta;
    delta_theta = (delta_theta > M_PI) ? delta_theta - 2*M_PI : (delta_theta < -M_PI) ? delta_theta + 2*M_PI : delta_theta;
    dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
        
    double lin_sat = 0.4, theta_sat = 0.19;
    if(dist > 0.035) // Go to point
    {
        if(-M_PI_2 <= delta_theta && M_PI_2 >= delta_theta) // Go forward
        {
            v = (fabs(delta_theta) > 45.0*M_PI/180.0) ? 0.0 : max(-lin_sat, min(lin_sat, (1.5 * dist)));
            omega = max(-theta_sat, min(theta_sat, (0.25 * delta_theta) + 0.45*(delta_theta - cvs->state->errorAngle)/(cvs->inputs->t - cvs->state->lastT)));
        }
        else
        {
            delta_theta = (delta_theta < 0) ? delta_theta + M_PI : delta_theta - M_PI;
            v = (fabs(delta_theta) > 45.0*M_PI/180.0) ? 0.0 : -max(-lin_sat, min(lin_sat, (1.5 * dist)));
            omega = max(-theta_sat, min(theta_sat, (0.25 * delta_theta) + 0.45*(delta_theta - cvs->state->errorAngle)/(cvs->inputs->t - cvs->state->lastT)));
        }
    }
    else    // Orientation to objective
    {
        delta_theta = destination[2] - theta;
        delta_theta = (delta_theta > M_PI) ? (delta_theta - 2*M_PI) : (delta_theta < -M_PI) ? (delta_theta + 2*M_PI) : delta_theta;
        v = 0;
//        omega = 0.3 * delta_theta;
//        omega = (omega > 0.15*M_PI_2) ? (0.15*M_PI_2) : (omega);
//        omega = (omega < -0.15*M_PI_2) ? (-0.15*M_PI_2) : (omega);
        if(fabs(delta_theta)*180.0/M_PI > 22.5)
            omega = max(-theta_sat, min(theta_sat, (0.375 * delta_theta)));
        else
            omega = max(-theta_sat, min(theta_sat, (0.25 * delta_theta) + 0.45*(delta_theta - cvs->state->errorAngle)/(cvs->inputs->t - cvs->state->lastT)));
    }
    
    // Check if opponent is on trajectory
    if(v > 0 && cvs->state->nb_opponents_detected != 0)
    {
        double x0, y0, x1, y1;
        x0 = cvs->state->opponent_position[0]; y0 = cvs->state->opponent_position[1];
        x1 = cvs->state->opponent_position[2]; y1 = cvs->state->opponent_position[3];
        
        dist = min(sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0)),sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1)));
        if(dist < 0.75)
            v = 0.0;
        if(cvs->state->opponent_timer == -42.0)
            cvs->state->opponent_timer = cvs->inputs->t;
    }
    else if (v < 0 && cvs->state->nb_opponents_detected != 0)
    {
        double x2, y2, x3, y3;
        x2 = cvs->state->opponent_position[4]; y2 = cvs->state->opponent_position[5];
        x3 = cvs->state->opponent_position[6]; y3 = cvs->state->opponent_position[7];
        
        dist = min(sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),sqrt((x-x3)*(x-x3)+(y-y3)*(y-y3)));
        if(dist < 0.6)
            v = 0.0;
        if(cvs->state->opponent_timer == -42.0)
            cvs->state->opponent_timer = cvs->inputs->t;
    }
    if((cvs->inputs->t - cvs->state->opponent_timer > 3) && cvs->state->opponent_timer != -42.0)
    {
        cvs->state->opponent_timer = -42.0;
        cvs->state->objectives[cvs->state->current_objective] = DELAYED;
    }
    if(cvs->state->nb_opponents_detected == 0)
        cvs->state->opponent_timer = -42.0;
    
    wheels[R_ID] = (v + omega)/0.0325;
    wheels[L_ID] = (v - omega)/0.0325;
    cvs->state->errorAngle = delta_theta;
    cvs->state->prev_theta = theta;
}
#endif