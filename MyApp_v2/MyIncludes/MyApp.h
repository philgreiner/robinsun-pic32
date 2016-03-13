/*******************************************************************************
* Header file for MyApp                                                        *
*******************************************************************************/

#ifndef MyAPP_H_
#define MyAPP_H_

// Adds support for PIC32 Peripheral library functions and macros
#include <plib.h>

// My Includes
#include "MySPI.h"
#include "MyLED.h"
#include "MyCyclone.h"
#include "MyConsole.h"
#include "MyCAN.h"
#include "MyRTCC.h"
#include "MyIO.h"
#include "MyFlash.h"
#include "MyMIWI.h"
#include "MyWIFI.h"
#include "MyTemperature.h"
#include "MyMDDFS.h"
#include "MyCamera.h"
#include "MyMiniProject.h"
#include "MyCtrlIO.h"
#include "ctrlstruct.h"

/*******************************************************************************
* Constants                                                                    *
*******************************************************************************/

// Application defines
#define SYS_FREQ		(80000000)

// Board defines
#define  CS_FPGA    BIT_4   // Port E
#define  CS_WIFI    BIT_3
#define  CS_MIWI    BIT_2
#define  CS_IO      BIT_1
#define  CS_FLASH   BIT_0

#define  RST_IO     BIT_5   // Port E
#define  RST_MIWI   BIT_6
#define  RST_WIFI   BIT_7

#define  RST_FPGA   BIT_7   // Port D
#define  GPIO_211   BIT_6
#define  GPIO_212   BIT_5
#define  GPIO_2IN0  BIT_4

#define  USD_CD     BIT_15  // Port B
#define  CS_USD     BIT_13
#define  CS_TEMP    BIT_12

#define  MIWI_WAKE  BIT_5   // Port B
#define  WIFI_WP    BIT_4
#define  WIFI_HIB   BIT_3
#define  SDN_5V     BIT_2
#define  FLASH_WP   BIT_6
#define  FLASH_HOLD BIT_7


/*******************************************************************************
* My Priorities for Interrupts                                                 *
*******************************************************************************/

#define My_INT_CAN_1_PRIORITY           INT_PRIORITY_LEVEL_2
#define My_INT_CAN_1_SUB_PRIORITY       INT_SUB_PRIORITY_LEVEL_0
#define My_INT_CAN_1_IPL                ipl2

#define My_INT_RTCC_PRIORITY            INT_PRIORITY_LEVEL_2
#define My_INT_RTCC_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0
#define My_INT_RTCC_IPL                 ipl2

#define My_INT_UART_PRIORITY            INT_PRIORITY_LEVEL_3
#define My_INT_UART_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0
#define My_INT_UART_IPL                 ipl3

#define My_INT_EXTERNAL_0_PRIORITY      INT_PRIORITY_LEVEL_5
#define My_INT_EXTERNAL_0_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_0_IPL           ipl5

#define My_INT_EXTERNAL_1_PRIORITY      INT_PRIORITY_LEVEL_6
#define My_INT_EXTERNAL_1_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_1_IPL           ipl6

#define My_INT_EXTERNAL_2_PRIORITY      INT_PRIORITY_LEVEL_1
#define My_INT_EXTERNAL_2_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_2_IPL           ipl1

#define My_INT_EXTERNAL_3_PRIORITY      INT_PRIORITY_LEVEL_4        // Not Modify ! (MIWI)
#define My_INT_EXTERNAL_3_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_3_IPL           ipl4

#define My_INT_EXTERNAL_4_PRIORITY      INT_PRIORITY_LEVEL_7        // Not Modify ! (WIFI)
#define My_INT_EXTERNAL_4_SUB_PRIORITY  INT_SUB_PRIORITY_LEVEL_0
#define My_INT_EXTERNAL_4_IPL           ipl7

/*******************************************************************************
* Global Variables                                                             *
*******************************************************************************/

#ifdef   MyAPP
#define  MyAPP_EXT
#define  MyAPP_INIT =FALSE
#else
#define  MyAPP_EXT  extern
#define  MyAPP_INIT
#endif

MyAPP_EXT  int      MyTime;
MyAPP_EXT  int      pbClk;
MyAPP_EXT  int      MyPing_Flag MyAPP_INIT;
MyAPP_EXT  int      MyMail_Flag MyAPP_INIT;

MyAPP_EXT  int      MyCyclone_LED;
MyAPP_EXT  int      MyCyclone_Switch;

MyAPP_EXT  int      MyVideo_Counter MyAPP_INIT;

/*******************************************************************************
* System Macros                                                                *
*******************************************************************************/

#define	GetSystemClock() 		(80000000ul)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())

/*******************************************************************************
* Functions Prototypes                                                         *
*******************************************************************************/


/*******************************************************************************
* Minibot global variables and constants                                       *
*******************************************************************************/

#define MotorSID 0x708
#define TowerSID 0x508

CtrlIn dataIN;
CtrlOut dataOUT;

MyAPP_EXT   float   Ki          MyAPP_INIT;
MyAPP_EXT   float   Kp          MyAPP_INIT;
MyAPP_EXT   float   speedREF    MyAPP_INIT;
MyAPP_EXT   float   maxspeed    MyAPP_INIT;
MyAPP_EXT   float   erreurIntR  MyAPP_INIT;
MyAPP_EXT   float   erreurIntL  MyAPP_INIT;
MyAPP_EXT   double  prevT       MyAPP_INIT;
#define     T_DISCRETISATION    10;  //msec
//#define     DEBUG

int i;
#endif /* MyAPP_H_ */
