/*! 
 * \author Philippe Greiner
 * \file MySubroutines.c
 * \brief Functions to easily send instructions to motors 
 *          and simplify the robot control.
 */

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
    #ifdef ROBOTCONSOLE
        char msg[1024];
        sprintf(msg, "Real duty: %d; MSB: %d; LSB: %d\n",realduty,*MSB,*LSB);
        MyConsole_SendMsg(msg);
    #endif
}

void    omegaRef(double *omegaref)
{
//    double beaconAngle = dataIN.angle;
//    double beaconDistance = dataIN.distance;
//    dataIN.tower_speed = beaconDistance; // to display on website
//    
//    double refspeed = speedREF/5.9; // SpeedREF is in cm/s; converted to rad/s
//    double speed = (beaconDistance<=20)? (0) : (refspeed + refspeed*(beaconDistance - 20));
//    if(beaconDistance < 10 && beaconDistance > 1) speed = -2*refspeed;
//    
//    speed = (speed > maxspeed)? (maxspeed) : (speed);
//    omegaref[0] = speed + speed*beaconAngle/90.0;
//    omegaref[1] = speed - speed*beaconAngle/90.0;
//    
//    //limitations
//    omegaref[0] = (omegaref[0]>maxspeed)? (maxspeed) : (omegaref[0]);
//    omegaref[1] = (omegaref[1]>maxspeed)? (maxspeed) : (omegaref[1]);
//    omegaref[0] = (omegaref[0]<-maxspeed)? (-maxspeed) : (omegaref[0]);
//    omegaref[1] = (omegaref[1]<-maxspeed)? (-maxspeed) : (omegaref[1]);
    
     if(sin(i*1.0/50.0) < 0)
     {
        omegaref[R_ID] = 6.3;
        omegaref[L_ID] = 6.3;
     }
     else
     {
        omegaref[R_ID] = -6.3;
        omegaref[L_ID] = -6.3;
     }
    
}