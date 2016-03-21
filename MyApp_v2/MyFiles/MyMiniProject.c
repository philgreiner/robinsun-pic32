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
/* \brief Initialize mobility */   
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

/* \brief Initialize fish motors */   
    MyCAN_TxMsg(FishSID, GPLAT);

    MyCAN_TxMsg(FishSID,T1CON);
    MyCAN_TxMsg(FishSID,T2CON);
    MyCAN_TxMsg(FishSID,PR1);
    MyCAN_TxMsg(FishSID,PR2);
    
    MyCAN_TxMsg(FishSID,MSBL);
    MyCAN_TxMsg(FishSID,LSBL);
        
    MyCAN_TxMsg(FishSID,MSBR);
    MyCAN_TxMsg(FishSID,LSBR);
    
/* \brief Initialize blocks motor */   
    char TOWERinit[3] = {0x1e, 0x70, 0x40}; //Turn ON LED (GP6) and turn OFF brakes (GP4)
    MyCAN_TxMsg(BlockSID, TOWERinit);
    
    char T1CONtow[3] = {0x21,0xb3,0x83}; //T1CON: TMR1ON = 1, -, Prescale1 = 00, --, DC1LSBs = 11
    char PR1tow[3] = {0x23,0xff,0x3f}; //PR1:   PR1 = 0x3F (f=78kHz)
    MyCAN_TxMsg(BlockSID,T1CONtow);
    MyCAN_TxMsg(BlockSID,PR1tow);
    
    unsigned LSBtow, MSBtow;
    DutyToInf(0.0, &MSBtow, &LSBtow);
    char LSBt[3] = {0x21, 0x03, LSBtow};
    char MSBt[3] = {0x25, 0x3f, MSBtow};
    MyCAN_TxMsg(BlockSID, LSBt);
    MyCAN_TxMsg(BlockSID, MSBt);
}

void    MyMiniProject_Update(CtrlStruct *cvs)
{
    unsigned int speedR = MyCyclone_Read(A_speedR);
    int dirR    = (MyCyclone_Read(A_dirR))*(-2)+1; // == 1 if going forward, == -1 otherwise
    cvs->inputs->r_wheel_speed = (double) -dirR*speedR*2*M_PI/(53242*.1); // in rad/s 

    unsigned int speedL = MyCyclone_Read(A_speedL);
    int dirL    = (MyCyclone_Read(A_dirL))*(2)-1; // == 1 if going forward, == -1 otherwise
    cvs->inputs->l_wheel_speed = (double) -dirL*speedL*2*M_PI/(47283*.1)*1.7; 
    
#ifdef ROBINSUN
    unsigned int speedOdoR = MyCyclone_Read(A_speedOdoR);
    int dirOdoR = (MyCyclone_Read(A_dirOdoR))*(-2)+1; // == 1 if going forward, == -1 otherwise
    cvs->inputs->odo_r_speed = (double) dirOdoR*speedOdoR*2*M_PI/(8192*.015);
    
    unsigned int speedOdoL = MyCyclone_Read(A_speedOdoL);
    int dirOdoL = (MyCyclone_Read(A_dirOdoL))*(2)-1; // == 1 if going forward, == -1 otherwise
    cvs->inputs->odo_l_speed = (double) dirOdoL*speedOdoL*2*M_PI/(8192*.015);
    
    unsigned int sonar12 = MyCyclone_Read(A_sonar12);
    unsigned int sonar34 = MyCyclone_Read(A_sonar34);
    unsigned int sonar56 = MyCyclone_Read(A_sonar56);
    cvs->inputs->sonars[0] = sonar12 &= 0xff00;
    cvs->inputs->sonars[1] = sonar12 &= 0x00ff;
    cvs->inputs->sonars[3] = sonar34 &= 0xff00;
    cvs->inputs->sonars[4] = sonar34 &= 0x00ff;
    cvs->inputs->sonars[5] = sonar56 &= 0xff00;
    cvs->inputs->sonars[6] = sonar56 &= 0x00ff;
#endif
    cvs->inputs->t = (ReadCoreTimer()/(SYS_FREQ/2.0)) - MyMiniProject_tStart; // time in seconds
}


void    MyMiniProject_Send(CtrlStruct *cvs)
{
    unsigned MSB0, LSB0;
    
    // Right motor
    DutyToInf(cvs->outputs->wheel_commands[R_ID], &MSB0, &LSB0);
    char MSBR[3] = {0x26, 0x3f, MSB0};
    char LSBR[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBR);
    MyCAN_TxMsg(MotorSID,LSBR);

    // Left motor
    DutyToInf(cvs->outputs->wheel_commands[L_ID], &MSB0, &LSB0);
    char MSBL[3] = {0x25, 0x3f, MSB0};
    char LSBL[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBL);
    MyCAN_TxMsg(MotorSID,LSBL);

    // Vertical fish motor
    DutyToInf(cvs->outputs->command_fish_vertical, &MSB0, &LSB0);
    char MSBFV[3] = {0x25, 0x3f, MSB0};
    char LSBFV[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(FishSID,MSBFV);
    MyCAN_TxMsg(FishSID,LSBFV);
    
    // Horizontal fish motor
    DutyToInf(cvs->outputs->command_fish_horizontal, &MSB0, &LSB0);
    char MSBFH[3] = {0x26, 0x3f, MSB0};
    char LSBFH[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(FishSID,MSBFH);
    MyCAN_TxMsg(FishSID,LSBFH);
    
    // Blocks motor
    DutyToInf(cvs->outputs->command_blocks, &MSB0, &LSB0);
    char MSBB[3] = {0x25, 0x3f, MSB0};
    char LSBB[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(BlockSID,MSBB);
    MyCAN_TxMsg(BlockSID,LSBB);
}


void MyMiniProject_Task(void)
{
#ifndef ROBINSUN
    MyDataUpdate();
    MyCtrlUpdate();
    SendCtrls();
#endif
}

/******************************************************************************/