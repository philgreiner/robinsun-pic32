/*******************************************************************************
* MyAnalog                                                                     *
********************************************************************************
* Description:                                                                 *
* Functions to read the analog inputs of the PIC32 microcontroller             *
********************************************************************************
* Version : 1.00 - March 2016                                                  *
*******************************************************************************/

#define  MyAnalog

#include "MyApp.h"

void MyAnalog_Init(void)
{    
    // configure and enable the ADC
    CloseADC10();	// ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // 				Turn module on | ouput in integer | trigger mode auto | enable autosample
    #define PARAM1  ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
    int PARAM1_2	 = ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON;

    // define setup parameters for OpenADC10
    // 				ADC ref external    | disable offset test    | disable scan mode | perform 2 samples | use dual buffers | use alternate mode
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON

    // define setup parameters for OpenADC10
    //				  use ADC internal clock | set sample time
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

    // define setup parameters for OpenADC10
    //               set AN4 and AN5 as analog inputs
    #define PARAM4	ENABLE_AN4_ANA | ENABLE_AN5_ANA

    // define setup parameters for OpenADC10
    // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL

    // use ground as neg ref for A | use AN4 for input A      |  use ground as neg ref for A | use AN5 for input B
    // configure to sample AN4 & AN5
    //SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 |  ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN3); // configure to sample AN4 & AN5
    SetChanADC10( ADC_CH0_POS_SAMPLEA_AN3 | ADC_CH0_POS_SAMPLEA_AN1 |  ADC_CH0_POS_SAMPLEA_AN2 | ADC_CH0_POS_SAMPLEB_AN0); // configure to sample AN4 & AN5

    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC
}

void MyAnalog_Read(unsigned int * AN1, unsigned int * AN3)
{ 
    if (mAD1GetIntFlag() ) 
    { 
        // wait for the first conversion to complete so there will be vaild data in ADC result registers
    
    unsigned int offset;	// buffer offset to point to the base of the idle buffer

    // the results of the conversions are available in channel4 and channel5
    offset = 8 * ((~ReadActiveBufferADC10() & 0x01));  // determine which buffer is idle and create an offset

    *AN1 = ReadADC10(offset);  		// read the result of channel AN1 conversion from the idle buffer
    *AN3 = ReadADC10(offset + 1);  	// read the result of channel AN3 conversion from the idle buffer

    char msg[1024];
    sprintf(msg, "AN1: %d, AN3: %d\n", *AN1, *AN3);
    MyConsole_SendMsg(msg);
    
    mAD1ClearIntFlag();
    }
    else 
    {
        *AN1 = 42;
        *AN3 = 42;
    }
}