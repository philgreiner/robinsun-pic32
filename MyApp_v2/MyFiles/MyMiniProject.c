/*******************************************************************************
* MyMiniProject                                                                *
********************************************************************************
* Description: Functions to control the Robot of the MiniProject 2.0           *
********************************************************************************
* Author : Philippe Greiner                                                    *
*******************************************************************************/

#define  MyMiniProject

#include "MyApp.h"
#include <math.h>

void MyMiniProject_Init(void)
{
//     Turn LED on (GP6)
//     Turn off the brakes (via the relevant GPIO's)
//     Use the mask to write only the needed registers (GP4, GP5 and GP6) 
    char GPLAT[3] = {0x1e, 0x70, 0x40};
    MyCAN_TxMsg(MotorSID, GPLAT);

    //Configure PWM Channels as desired
    char T1CON[3] = {0x21,0xb3,0x80}; //T1CON: TMR1ON = 1, -, Prescale1 = 00, --, DC1LSBs = 11
    char T2CON[3] = {0x22,0xb3,0x80}; //T2CON: TMR2ON = 1, -, Prescale2 = 00, --, DC2LSBs = 11
    char PR1[3] = {0x23,0xff,0x3f}; //PR1:   PR1 = 0x3F (f=78kHz)
    char PR2[3] = {0x24,0xff,0x3f}; //PR2:   PR2 = 0x3F (f=78kHz)

    MyCAN_TxMsg(MotorSID,T1CON);
    MyCAN_TxMsg(MotorSID,T2CON);
    MyCAN_TxMsg(MotorSID,PR1);
    MyCAN_TxMsg(MotorSID,PR2);
    
    unsigned MSB0, LSB0;
    DutyToInf(0, &MSB0, &LSB0);
    char MSBL[3] = {0x26, 0x3f, MSB0};
    char LSBL[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBL);
    MyCAN_TxMsg(MotorSID,LSBL);
        
    char MSBR[3] = {0x25, 0x3f, MSB0};
    char LSBR[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBR);
    MyCAN_TxMsg(MotorSID,LSBR);

////    MyDelayMs(1000);
//    char TOWERinit[3] = {0x1e, 0x70, 0x40}; //Turn ON LED (GP6) and turn OFF brakes (GP4)
//    MyCAN_TxMsg(TowerSID, TOWERinit);
//    
//    char T1CONtow[3] = {0x21,0xb3,0x83}; //T1CON: TMR1ON = 1, -, Prescale1 = 00, --, DC1LSBs = 11
//    char PR1tow[3] = {0x23,0xff,0x3f}; //PR1:   PR1 = 0x3F (f=78kHz)
//    MyCAN_TxMsg(TowerSID,T1CONtow);
//    MyCAN_TxMsg(TowerSID,PR1tow);
//    
//    unsigned LSBtow, MSBtow;
//    DutyToInf(10.0, &MSBtow, &LSBtow);
//    char LSBt[3] = {0x21, 0x03, LSBtow};
//    char MSBt[3] = {0x25, 0x3f, MSBtow};
//    MyCAN_TxMsg(TowerSID, LSBt);
//    MyCAN_TxMsg(TowerSID, MSBt);
//    
    Kp = 0.09;//0.095;
    Ki = 19.71;//14.25;
    speedREF = 10;
    maxspeed = 20.0;
    erreurIntR = 0.0;
    erreurIntL = 0.0;
    
    dataIN.l_wheel_speed = 0.0;
    dataIN.r_wheel_speed = 0.0;
    
    i = 0;
}

void    MyMiniProject_Update(CtrlStruct *cvs)
{
    unsigned int speedR1 = MyCyclone_Read(A_speedR1);
    unsigned int speedR2 = MyCyclone_Read(A_speedR2);
    int dirR    = (MyCyclone_Read(A_dirR))*(-2)+1; // == 1 if going forward, == -1 otherwise
    cvs->inputs->r_wheel_speed = (double) dirR*(speedR1*256.0 + speedR2)*2*M_PI/(53242*.015); // in rad/s 

    unsigned int speedL1 = MyCyclone_Read(A_speedL1);
    unsigned int speedL2 = MyCyclone_Read(A_speedL2);
    int dirL    = (MyCyclone_Read(A_dirL))*(2)-1; // == 1 if going forward, == -1 otherwise
    cvs->inputs->l_wheel_speed = (double) dirL*(speedL1*256.0 + speedL2)*2*M_PI/(47283*.015); 
    
    cvs->inputs->t = (ReadCoreTimer()/(SYS_FREQ/2.0));; // time in seconds
}


void    MyMiniProject_Send(CtrlStruct *cvs)
{
    unsigned MSB0, LSB0;
    
    DutyToInf(cvs->outputs->wheel_commands[R_ID], &MSB0, &LSB0);
    char MSBR[3] = {0x26, 0x3f, MSB0};
    char LSBR[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBR);
    MyCAN_TxMsg(MotorSID,LSBR);

    #ifdef ROBOTCONSOLE
        char msg[1024];
        sprintf(msg, "Right command: MSB: %d LSB: %d\n",MSB0*4,LSB0);
        MyConsole_SendMsg(msg);
    #endif

    DutyToInf(cvs->outputs->wheel_commands[L_ID], &MSB0, &LSB0);
    char MSBL[3] = {0x25, 0x3f, MSB0};
    char LSBL[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBL);
    MyCAN_TxMsg(MotorSID,LSBL);

    #ifdef ROBOTCONSOLE
        sprintf(msg, "Left command: MSB: %d LSB: %d\n\n",MSB0*4,LSB0);
        MyConsole_SendMsg(msg);
    #endif
}


void MyMiniProject_Task(void)
{
    MyDataUpdate();
    MyCtrlUpdate();
    SendCtrls();
}

/******************************************************************************/