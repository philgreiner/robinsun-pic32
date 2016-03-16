/*! 
 * \author Philippe Greiner
 * \file MyCtrlIO.c
 * \brief Functions to update the data collected by the FPGA to the PIC32.
 */
#define MyCtrlIO

#include "MyApp.h"
#include "MyCtrlIO.h"
#include <math.h>
#ifndef ROBINSUN

/* Function that updates all the data collected by the FPGA:
 *      --> both wheel speeds
 *      --> laser tower speed
 *      --> beacon angle opening
 *      --> beacon angle relative to minibot
 */
void MyDataUpdate(void)
{
    unsigned int speedR1 = MyCyclone_Read(A_speedR1);
    unsigned int speedR2 = MyCyclone_Read(A_speedR2);
    int dirR    = (MyCyclone_Read(A_dirR))*(-2)+1; // == 1 if going forward, == -1 otherwise
    dataIN.r_wheel_speed = (double) dirR*(speedR1*256.0 + speedR2)*2*M_PI/(53242*.015); // in rad/s 

    unsigned int speedL1 = MyCyclone_Read(A_speedL1);
    unsigned int speedL2 = MyCyclone_Read(A_speedL2);
    int dirL    = (MyCyclone_Read(A_dirL))*(2)-1; // == 1 if going forward, == -1 otherwise
    dataIN.l_wheel_speed = (double) dirL*(speedL1*256.0 + speedL2)*2*M_PI/(47283*.015); 
}

// Motors speed controller
void MyCtrlUpdate(void)
{
    double omegaref[2];
    double UconsigneR, UconsigneL;
    double Valim = 24.0;
    
    // Get values for current speed and reference speed
    omegaRef(&omegaref);  // [rad/s]
    double rspeed = dataIN.r_wheel_speed;
    double lspeed = dataIN.l_wheel_speed;
    
    double t = (ReadCoreTimer()/(SYS_FREQ/2000.0));
    
    // Integrate the error
    erreurIntR += (omegaref[R_ID]-rspeed)*(t - prevT)*0.001; 
    erreurIntL += (omegaref[L_ID]-lspeed)*(t - prevT)*0.001;
    
    // Limit the integral error (anti-windup)
    erreurIntR = (erreurIntR*Ki>Valim) ? ( Valim/(Ki)) : (erreurIntR);
    erreurIntL = (erreurIntL*Ki>Valim) ? ( Valim/(Ki)) : (erreurIntL);
    erreurIntR = (erreurIntR*Ki<-Valim)? (-Valim/(Ki)) : (erreurIntR);
    erreurIntL = (erreurIntL*Ki<-Valim)? (-Valim/(Ki)) : (erreurIntL);

    // PI controller
    UconsigneR = (omegaref[R_ID]-rspeed)*Kp + erreurIntR*Ki;
    UconsigneL = (omegaref[L_ID]-lspeed)*Kp + erreurIntL*Ki;

    UconsigneR = (UconsigneR>Valim) ? ( Valim) : (UconsigneR);
    UconsigneR = (UconsigneR<-Valim)? (-Valim) : (UconsigneR);
    UconsigneL = (UconsigneL>Valim) ? ( Valim) : (UconsigneL);
    UconsigneL = (UconsigneL<-Valim)? (-Valim) : (UconsigneL);
    
//    char msg[1024];
//    sprintf(msg, "Right speed: %.3f; omegaref: %.3f; P = %.3f, I = %.3f, UconsigneR: %.3f;\n",rspeed, omegaref[R_ID], (omegaref[R_ID]-rspeed), erreurIntR, UconsigneR);
//    MyConsole_SendMsg(msg);
//    sprintf(msg, "Left speed: %.3f; omegaref: %.3f; P = %.3f, I = %.3f, UconsigneR: %.3f;\n",lspeed, omegaref[L_ID], (omegaref[L_ID]-lspeed), erreurIntL, UconsigneL);
//    MyConsole_SendMsg(msg);
    
    #ifdef ROBOTCONSOLE
        char msg[1024];
        sprintf(msg, "Right speed: %.3f; omegaref: %.3f; UconsigneR: %.3f\n",rspeed,omegaref[R_ID], UconsigneR);
        MyConsole_SendMsg(msg);
    #endif

    // Update command values
    dataOUT.wheel_commands[R_ID] = 100.0*UconsigneR/26.0;
    dataOUT.wheel_commands[L_ID] = 100.0*UconsigneL/26.0;
    i++;
    prevT = (ReadCoreTimer()/(SYS_FREQ/2000.0));
}

// Apply command values
void SendCtrls(void)
{
    unsigned MSB0, LSB0;
    
    DutyToInf(dataOUT.wheel_commands[R_ID], &MSB0, &LSB0);
    char MSBR[3] = {0x26, 0x3f, MSB0};
    char LSBR[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBR);
    MyCAN_TxMsg(MotorSID,LSBR);

    #ifdef ROBOTCONSOLE
        char msg[1024];
        sprintf(msg, "Right command: MSB: %d LSB: %d\n",MSB0*4,LSB0);
        MyConsole_SendMsg(msg);
    #endif

    DutyToInf(dataOUT.wheel_commands[L_ID], &MSB0, &LSB0);
    char MSBL[3] = {0x25, 0x3f, MSB0};
    char LSBL[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBL);
    MyCAN_TxMsg(MotorSID,LSBL);

    #ifdef ROBOTCONSOLE
        sprintf(msg, "Left command: MSB: %d LSB: %d\n\n",MSB0*4,LSB0);
        MyConsole_SendMsg(msg);
    #endif
//    DutyToInf(dataOUT.tower_command, &MSB0, &LSB0);
//    char MSBtow[3] = {0x25, 0x3f, MSB0};
//    char LSBtow[3] = {0x21, 0x03, LSB0};
//    MyCAN_TxMsg(TowerSID,MSBtow);
//    MyCAN_TxMsg(TowerSID,LSBtow);
}
#endif