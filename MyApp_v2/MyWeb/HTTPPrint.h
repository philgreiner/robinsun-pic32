/**************************************************************
 * HTTPPrint.h
 * Provides callback headers and resolution for user's custom
 * HTTP Application.
 * 
 * This file is automatically generated by the MPFS Utility
 * ALL MODIFICATIONS WILL BE OVERWRITTEN BY THE MPFS GENERATOR
 **************************************************************/

#ifndef __HTTPPRINT_H
#define __HTTPPRINT_H

#include "TCPIP Stack/TCPIP.h"

#if defined(STACK_USE_HTTP2_SERVER)

extern HTTP_STUB httpStubs[MAX_HTTP_CONNECTIONS];
extern BYTE curHTTPID;

void HTTPPrint(DWORD callbackID);
void HTTPPrint_r_wheel_speed(void);
void HTTPPrint_l_wheel_speed(void);
void HTTPPrint_refspeed(void);
void HTTPPrint_buttonL(void);
void HTTPPrint_buttonR(void);
void HTTPPrint_omegaref_R(void);
void HTTPPrint_omegaref_L(void);
void HTTPPrint_sonar1(void);
void HTTPPrint_sonar2(void);
void HTTPPrint_sonar3(void);
void HTTPPrint_sonar4(void);
void HTTPPrint_sonar5(void);
void HTTPPrint_sonar6(void);
void HTTPPrint_speed_blocks(void);
void HTTPPrint_speed_fish_horizontal(void);
void HTTPPrint_speed_fish_vertical(void);
void HTTPPrint_command_blocks(void);
void HTTPPrint_command_fish_horizontal(void);
void HTTPPrint_command_fish_vertical(void);
void HTTPPrint_odometer_R(void);
void HTTPPrint_odometer_L(void);
void HTTPPrint_position0(void);
void HTTPPrint_position1(void);
void HTTPPrint_position2(void);
void HTTPPrint_goal0(void);
void HTTPPrint_goal1(void);
void HTTPPrint_goal2(void);
void HTTPPrint_competition_state(void);
void HTTPPrint_time(void);
void HTTPPrint_current_progress(void);
void HTTPPrint_kpleft(void);
void HTTPPrint_kpright(void);
void HTTPPrint_kileft(void);
void HTTPPrint_kiright(void);
void HTTPPrint_irL(void);
void HTTPPrint_irR(void);
void HTTPPrint_opponent0(void);
void HTTPPrint_opponent1(void);
void HTTPPrint_opponent2(void);
void HTTPPrint_opponent3(void);
void HTTPPrint_opponentsdetected(void);

void HTTPPrint(DWORD callbackID)
{
	switch(callbackID)
	{
        case 0x00000002:
			HTTPIncFile((ROM BYTE*)"footer.inc");
			break;
        case 0x00000033:
			HTTPIncFile((ROM BYTE*)"header.inc");
			break;
        case 0x0000004b:
			HTTPPrint_r_wheel_speed();
			break;
        case 0x0000004c:
			HTTPPrint_l_wheel_speed();
			break;
        case 0x00000061:
			HTTPPrint_refspeed();
			break;
        case 0x00000065:
			HTTPPrint_buttonL();
			break;
        case 0x00000066:
			HTTPPrint_buttonR();
			break;
        case 0x00000067:
			HTTPPrint_omegaref_R();
			break;
        case 0x00000068:
			HTTPPrint_omegaref_L();
			break;
        case 0x00000069:
			HTTPPrint_sonar1();
			break;
        case 0x0000006a:
			HTTPPrint_sonar2();
			break;
        case 0x0000006b:
			HTTPPrint_sonar3();
			break;
        case 0x0000006c:
			HTTPPrint_sonar4();
			break;
        case 0x0000006d:
			HTTPPrint_sonar5();
			break;
        case 0x0000006e:
			HTTPPrint_sonar6();
			break;
        case 0x0000006f:
			HTTPPrint_speed_blocks();
			break;
        case 0x00000070:
			HTTPPrint_speed_fish_horizontal();
			break;
        case 0x00000071:
			HTTPPrint_speed_fish_vertical();
			break;
        case 0x00000072:
			HTTPPrint_command_blocks();
			break;
        case 0x00000073:
			HTTPPrint_command_fish_horizontal();
			break;
        case 0x00000074:
			HTTPPrint_command_fish_vertical();
			break;
        case 0x00000075:
			HTTPPrint_odometer_R();
			break;
        case 0x00000076:
			HTTPPrint_odometer_L();
			break;
        case 0x00000077:
			HTTPPrint_position0();
			break;
        case 0x00000078:
			HTTPPrint_position1();
			break;
        case 0x00000079:
			HTTPPrint_position2();
			break;
        case 0x0000007a:
			HTTPPrint_goal0();
			break;
        case 0x0000007b:
			HTTPPrint_goal1();
			break;
        case 0x0000007c:
			HTTPPrint_goal2();
			break;
        case 0x0000007d:
			HTTPPrint_competition_state();
			break;
        case 0x0000007f:
			HTTPPrint_time();
			break;
        case 0x00000080:
			HTTPPrint_current_progress();
			break;
        case 0x00000081:
			HTTPPrint_kpleft();
			break;
        case 0x00000082:
			HTTPPrint_kpright();
			break;
        case 0x00000083:
			HTTPPrint_kileft();
			break;
        case 0x00000084:
			HTTPPrint_kiright();
			break;
        case 0x00000085:
			HTTPPrint_irL();
			break;
        case 0x00000086:
			HTTPPrint_irR();
			break;
        case 0x00000087:
			HTTPPrint_opponent0();
			break;
        case 0x00000088:
			HTTPPrint_opponent1();
			break;
        case 0x00000089:
			HTTPPrint_opponent2();
			break;
        case 0x0000008a:
			HTTPPrint_opponent3();
			break;
        case 0x0000008b:
			HTTPPrint_opponentsdetected();
			break;
		default:
			// Output notification for undefined values
			TCPPutROMArray(sktHTTP, (ROM BYTE*)"!DEF", 4);
	}

	return;
}

void HTTPPrint_(void)
{
	TCPPut(sktHTTP, '~');
	return;
}

#endif

#endif
