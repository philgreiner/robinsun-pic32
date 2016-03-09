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
void HTTPPrint_angle(void);
void HTTPPrint_distance(void);
void HTTPPrint_rref(void);
void HTTPPrint_lref(void);
void HTTPPrint_refspeed(void);
void HTTPPrint_maxspeed(void);
void HTTPPrint_kp(void);
void HTTPPrint_ki(void);
void HTTPPrint_buttonL(void);
void HTTPPrint_buttonR(void);

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
        case 0x00000052:
			HTTPPrint_rref();
			break;
        case 0x00000053:
			HTTPPrint_lref();
			break;
        case 0x00000061:
			HTTPPrint_refspeed();
			break;
        case 0x00000062:
			HTTPPrint_maxspeed();
			break;
        case 0x00000063:
			HTTPPrint_kp();
			break;
        case 0x00000064:
			HTTPPrint_ki();
			break;
        case 0x00000065:
			HTTPPrint_buttonL();
			break;
        case 0x00000066:
			HTTPPrint_buttonR();
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
