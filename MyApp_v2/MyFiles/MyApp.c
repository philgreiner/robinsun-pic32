/*******************************************************************************
* MyApp                                                                        *
********************************************************************************
* Description:                                                                 *
* This application controls the PIC32 of MyExpansionBoard_Pic32                *
********************************************************************************
* Version : 2.00 - Nov 2012                                                    *
* based on microchip_solutions_v2012-10-15                                     *
*******************************************************************************/

#define  MyAPP

#include "MyApp.h"

// Configuration Bits
// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = (SYSCLK / FPBDIV)

#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (PIC32 Starter Kit: use divide by 2 only)
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor : Clock Switching and FSCM are disabled
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = XT            // Primary Oscillator (3.5 MHz-10 MHz resonator, crystal or resonator)
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = ON            // Secondary Oscillator Enable
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select : PGEC1/PGED1 pair is used
#pragma config DEBUG    = OFF           // Background Debugger Enable

// My PIC-thread macros and vars below
unsigned char seq[2];  // will handle 2 threads
unsigned char thseq;
unsigned char thseqtemp;
#define THREAD_START(A) thseq=seq[A]; thseqtemp=0; if(thseq==thseqtemp) {
#define THREAD_BREAK } thseqtemp++; if(thseq==thseqtemp) {
#define THREAD_END(B) } seq[B]++; if(seq[B]>thseqtemp) seq[B]=0;

const char startMsg[] =
{
    "\r\n\r\n"\
    "Welcome to myApp !\r\n"\
    "The build date and time is (" __DATE__ "," __TIME__ ")\r\n"\
    "\r\n"\
    ">"
};


void MyDelayMs(unsigned int msec);


int main(void)
{
    // KEY CONCEPT - Configure the device for maximum performance, but do not change the PBDIV clock divisor.
    // Given the options, this function will change the program Flash wait states,
    // RAM wait state and enable prefetch cache, but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above.
    pbClk = SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // Set the CS signals to default value to avoid conflicts on the SPI
    // Port E : FPGA, Wifi, Miwi, IO, Flash
    // Port B : MicroSD, Temperature (CS is not inverted !)
    mPORTESetBits(CS_FPGA | CS_WIFI | CS_MIWI | CS_IO | CS_FLASH);
    mPORTESetPinsDigitalOut(CS_FPGA | CS_WIFI | CS_MIWI | CS_IO | CS_FLASH);

    mPORTBSetBits(CS_USD);
    mPORTBClearBits(CS_TEMP);
    mPORTBSetPinsDigitalOut(CS_TEMP | CS_USD);
    
    // Do MyInitializations
    MyConsole_Init();
    //MyCamera_Init();
    MyRTCC_Init();
    MyLED_Init();
    MySPI_Init();
    MyCyclone_Init();
    MyCAN_Init();
    MyIO_Init();
    MyFlash_Init();
    //MyTemperature_Init();
    //MyMIWI_Init();
//    MyWIFI_Init();

    // Configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    // Enable interrupts
    INTEnableInterrupts();

    // Display the Start Message
    MyConsole_SendMsg(startMsg);

    // Start Wireless and Camera connections
//    MyMIWI_Start();
//    MyWIFI_Start();
//    MyCamera_Start();

    MyMiniProject_Init();
    MyCAN_Task();

    CtrlIn *inputs = (CtrlIn*) malloc(sizeof(CtrlIn));
    CtrlOut *outputs = (CtrlOut*) malloc(sizeof(CtrlOut));
//    cvs = init_CtrlStruct(inputs, outputs);
    
    MyMiniProject_tStart = (ReadCoreTimer()/(SYS_FREQ/2.0));
//    MyMiniProject_Update(cvs);
//    controller_init(cvs);
    MyConsole_SendMsg("Attempting I2C begin();\n");
    int status = begin(0, MODE_INTERRUPT, POZYX_INT_MASK_ALL, POZYX_INT_PIN0);
    char msg[1024];
    sprintf(msg, "I2C begin() returned %d\n", status);
    MyConsole_SendMsg(msg);
    // Execute forever    
    while (1)
    {
        THREAD_START(1);
            MyConsole_Task();
        THREAD_BREAK
//            MyWIFI_Task();
        THREAD_END(1);
        THREAD_START(0);
            MyRTCC_Task();            
//            MyMiniProject_Update(cvs);
//            controller_loop(cvs);
//            MyMiniProject_Send(cvs);
            MyCAN_Task();
        THREAD_END(0);
    }
}

/******************************************************************************
*	MyDelayMs()
*
*	This functions provides a software millisecond delay
******************************************************************************/

void MyDelayMs(unsigned int msec)
{
	unsigned int tWait, tStart;

    tWait=(SYS_FREQ/2000)*msec;
    tStart=ReadCoreTimer();
    while((ReadCoreTimer()-tStart)<tWait);		// wait for the time to pass
}

// If your code gets here, you either tried to read or write
// a NULL pointer, or your application overflowed the stack
// by having too many local variables or parameters declared.

void _general_exception_handler(unsigned cause, unsigned status)
{
    //MyConsole_SendMsg("Exception.\n");
    Nop();
    Nop();
}