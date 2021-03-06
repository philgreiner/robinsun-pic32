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
    DutyToInf(0.0, &MSB0, &LSB0);
    char MSBL[3] = {0x26, 0x3f, MSB0};
    char LSBL[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBL);
    MyCAN_TxMsg(MotorSID,LSBL);
        
    char MSBR[3] = {0x25, 0x3f, MSB0};
    char LSBR[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBR);
    MyCAN_TxMsg(MotorSID,LSBR);
    
    MyDelayMs(10);

    #ifdef ROBINSUN
        /* \brief Initialize fish motors */   
        char GPLAT2[3] = {0x1e, 0x70, 0xff};
        MyCAN_TxMsg(FishSID, GPLAT2);

        MyCAN_TxMsg(FishSID,T1CON);
        MyCAN_TxMsg(FishSID,T2CON);
        MyCAN_TxMsg(FishSID,PR1);
        MyCAN_TxMsg(FishSID,PR2);
    
        MyCAN_TxMsg(FishSID,MSBL);
        MyCAN_TxMsg(FishSID,LSBL);
        
        MyCAN_TxMsg(FishSID,MSBR);
        MyCAN_TxMsg(FishSID,LSBR);

        MyDelayMs(10);
        
    /* \brief Initialize blocks motor */   
        char BLOCKSinit[3] = {0x1e, 0x70, 0x40}; //Turn ON LED (GP6) and turn OFF brakes (GP4)
        MyCAN_TxMsg(BlockSID, BLOCKSinit);
    
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
        MyDelayMs(10);
    #endif
}

void    MyMiniProject_Update(CtrlStruct *cvs)
{
    #ifdef ROBINSUN
        int speedR = MyCyclone_Read(A_speedR);
        speedR = ((speedR >> 15) == 1)? speedR-65535 : speedR;
        cvs->inputs->r_wheel_speed = (double) 60.0/61.92*30.0/22.0*speedR*2*M_PI*1.05/(53242*0.025); // 53242 in rad/s 

        int speedL = MyCyclone_Read(A_speedL);
        speedL = ((speedL >> 15) == 1)? speedL-65535 : speedL;
        cvs->inputs->l_wheel_speed = (double) -60.0/60.74*30.0/24.0*speedL*2*M_PI/(47283*0.025); 
    #endif
    
    #ifdef MINIBOT
        unsigned int speedR = MyCyclone_Read(A_speedR);
        int dirR    = (MyCyclone_Read(A_dirR))*(-2)+1; // == 1 if going forward, == -1 otherwise
        cvs->inputs->r_wheel_speed = (double) dirR*speedR*(1.0/255.0);

        unsigned int speedL = MyCyclone_Read(A_speedL);
        int dirL    = (MyCyclone_Read(A_dirL))*(2)-1; // == 1 if going forward, == -1 otherwise
        cvs->inputs->l_wheel_speed = (double) dirL*speedL*(1.0/112.5);
    #endif
   
    #ifdef ROBINSUN
        int speedOdoR = MyCyclone_Read(A_speedOdoR);
        speedOdoR = ((speedOdoR >> 15) == 1)? speedOdoR-65535 : speedOdoR;
        cvs->inputs->odo_r_speed = (double) -speedOdoR*2*M_PI/(8192*.025);
        
        int speedOdoL = MyCyclone_Read(A_speedOdoL);
        speedOdoL = ((speedOdoL >> 15) == 1)? speedOdoL-65535 : speedOdoL;
        cvs->inputs->odo_l_speed = (double) speedOdoL*2*M_PI/(8192*.025);
    
        unsigned int ADC = MyCyclone_Read(A_adc);
        cvs->inputs->irL = ((float) ((ADC & 0xff00) >> 8)) * 3.3/255.0;
        cvs->inputs->irR = ((float) (ADC & 0x00ff))        * 3.3/255.0;
        
        unsigned int lt24 = MyCyclone_Read(A_lt24);
        // Team color defined
        if(lt24%2)
            cvs->inputs->team_color = 1;
        else
            cvs->inputs->team_color = 0;
        
        // Strategy defined
//        if((lt24 >> 2)%2)
//            cvs->inputs->strategy = 1;
//        else 
        if(lt24 == 41)
            MyCyclone_Write(A_PICtoFPGA, 0x0001 & 1);
        cvs->inputs->strategy_cabins = (lt24 >> 2)%2;
        cvs->inputs->strategy_fish = (lt24 >> 3)%2;
        cvs->inputs->mobility = (lt24 >> 4)%2;
        cvs->inputs->strategy_blockscabins = (lt24 >> 5)%2;
        cvs->inputs->fish_fullIN = (lt24 >> 14)%2;
        cvs->inputs->fish_startIN = (lt24 >> 13)%2;
//        else if((lt24 >> 4)%2)
//            cvs->inputs->strategy = 3;
//        else
//            cvs->inputs->strategy = 4;
        
        // Number of opponents detected
        if((lt24 >> 6)%2)
            cvs->inputs->mode = 1;
        else
            cvs->inputs->mode = 0;
        
        
        // Are the data sent to ROBINSUN ?
        cvs->inputs->ready_signal = (lt24 >> 8)%2;
        cvs->inputs->start_signal = lt24 >> 15;
        
        unsigned int sonar12 = MyCyclone_Read(A_sonar12);
        unsigned int sonar34 = MyCyclone_Read(A_sonar34);
        unsigned int sonar56 = MyCyclone_Read(A_sonar56);
        cvs->inputs->sonars[0] = (sonar12 & 0xff00) >> 8;
        cvs->inputs->sonars[1] = (sonar12 & 0x00ff);
        cvs->inputs->sonars[2] = (sonar34 & 0xff00) >> 8;
        cvs->inputs->sonars[3] = (sonar34 & 0x00ff);
        cvs->inputs->sonars[4] = (sonar56 & 0xff00) >> 8;
        cvs->inputs->sonars[5] = (sonar56 & 0x00ff);
        
        //locate_opponent(cvs);
    #endif
    cvs->inputs->t = (ReadCoreTimer()/(SYS_FREQ/2.0)) - MyMiniProject_tStart; // time in seconds
    while (cvs->inputs->t < cvs->state->lastT) cvs->inputs->t += 107.3741823075;
}

void    MyMiniProject_Send(CtrlStruct *cvs)
{
    unsigned MSB0, LSB0;
    
    // Right motor
    DutyToInf(cvs->outputs->wheel_commands[R_ID], &MSB0, &LSB0);
    char MSBR[3] = {0x25, 0x3f, MSB0};
    char LSBR[3] = {0x21, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBR);
    MyCAN_TxMsg(MotorSID,LSBR);

    // Left motor
    DutyToInf(cvs->outputs->wheel_commands[L_ID], &MSB0, &LSB0);
    char MSBL[3] = {0x26, 0x3f, MSB0};
    char LSBL[3] = {0x22, 0x03, LSB0};
    MyCAN_TxMsg(MotorSID,MSBL);
    MyCAN_TxMsg(MotorSID,LSBL);
    
    #ifdef ROBINSUN
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
    #endif
}

/******************************************************************************/