/*********************************************************************
 *
 *  Application to Demo HTTP2 Server
 *  Support for HTTP2 module in Microchip TCP/IP Stack
 *	 -Implements the application 
 *	 -Reference: RFC 1002
 *
 ********************************************************************/
#define MyHTTP

#include "TCPIPConfig.h"
#include "myprints.h"

#if defined(STACK_USE_HTTP2_SERVER)

#include "TCPIP Stack/TCPIP.h"
#include "MyApp.h"
#include <math.h>

#define LED0_IO     (LATBbits.LATB9)
#define LED1_IO     (LATBbits.LATB9)
#define LED2_IO     (LATBbits.LATB9)
#define LED3_IO     (LATBbits.LATB9)
#define LED4_IO     (LATBbits.LATB10)
#define LED5_IO     (LATBbits.LATB10)
#define LED6_IO     (LATBbits.LATB10)
#define LED7_IO     (LATBbits.LATB10)

#define BUTTON0_IO  (LATBbits.LATB9)
#define BUTTON1_IO  (LATBbits.LATB9)
#define BUTTON2_IO  (LATBbits.LATB10)
#define BUTTON3_IO  (LATBbits.LATB10)

#define POST_READ   (0u)
#define POST_DONE   (1u)

/****************************************************************************
  Section:
	Function Prototypes and Memory Globalizers
  ***************************************************************************/
static HTTP_IO_RESULT HTTPautocontrol(void);

#if defined(HTTP_USE_POST)
	#if defined(USE_LCD)
		static HTTP_IO_RESULT HTTPPostLCD(void);
	#endif
	#if defined(STACK_USE_HTTP_MD5_DEMO)
		#if !defined(STACK_USE_MD5)
			#error The HTTP_MD5_DEMO requires STACK_USE_MD5
		#endif
		static HTTP_IO_RESULT HTTPPostMD5(void);
	#endif
	#if defined(STACK_USE_HTTP_APP_RECONFIG)
		extern APP_CONFIG AppConfig;
		static HTTP_IO_RESULT HTTPPostConfig(void);
		#if defined(STACK_USE_SNMP_SERVER)
		static HTTP_IO_RESULT HTTPPostSNMPCommunity(void);
		#endif
	#endif
	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
		static HTTP_IO_RESULT HTTPPostDDNSConfig(void);
	#endif
#endif

// RAM allocated for DDNS parameters
#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	static BYTE DDNSData[100];
#endif

// Sticky status message variable.
// This is used to indicated whether or not the previous POST operation was 
// successful.  The application uses these to store status messages when a 
// POST operation redirects.  This lets the application provide status messages
// after a redirect, when connection instance data has already been lost.
static BOOL lastSuccess = FALSE;

// Stick status message variable.  See lastSuccess for details.
static BOOL lastFailure = FALSE;

/****************************************************************************
  Section:
	Authorization Handlers
  ***************************************************************************/
  
/*****************************************************************************
  Function:
	BYTE HTTPNeedsAuth(BYTE* cFile)
	
  Internal:
  	See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
#if defined(HTTP_USE_AUTHENTICATION)
BYTE HTTPNeedsAuth(BYTE* cFile)
{
	// If the filename begins with the folder "protect", then require auth
	if(memcmppgm2ram(cFile, (ROM void*)"protect", 7) == 0)
		return 0x00;		// Authentication will be needed later

	// If the filename begins with the folder "snmp", then require auth
	if(memcmppgm2ram(cFile, (ROM void*)"snmp", 4) == 0)
		return 0x00;		// Authentication will be needed later

	#if defined(HTTP_MPFS_UPLOAD_REQUIRES_AUTH)
	if(memcmppgm2ram(cFile, (ROM void*)"mpfsupload", 10) == 0)
		return 0x00;
	#endif

	// You can match additional strings here to password protect other files.
	// You could switch this and exclude files from authentication.
	// You could also always return 0x00 to require auth for all files.
	// You can return different values (0x00 to 0x79) to track "realms" for below.

	return 0x80;			// No authentication required
}
#endif

/*****************************************************************************
  Function:
	BYTE HTTPCheckAuth(BYTE* cUser, BYTE* cPass)
	
  Internal:
  	See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
#if defined(HTTP_USE_AUTHENTICATION)
BYTE HTTPCheckAuth(BYTE* cUser, BYTE* cPass)
{
	if(strcmppgm2ram((char *)cUser,(ROM char *)"admin") == 0
		&& strcmppgm2ram((char *)cPass, (ROM char *)"microchip") == 0)
		return 0x80;		// We accept this combination
	
	// You can add additional user/pass combos here.
	// If you return specific "realm" values above, you can base this 
	//   decision on what specific file or folder is being accessed.
	// You could return different values (0x80 to 0xff) to indicate 
	//   various users or groups, and base future processing decisions
	//   in HTTPExecuteGet/Post or HTTPPrint callbacks on this value.
	
	return 0x00;			// Provided user/pass is invalid
}
#endif

/****************************************************************************
  Section:
	GET Form Handlers
  ***************************************************************************/
  
/*****************************************************************************
  Function:
	HTTP_IO_RESULT HTTPExecuteGet(void)
	
  Internal:
  	See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
HTTP_IO_RESULT HTTPExecuteGet(void)
{
	BYTE *ptr;
	BYTE filename[20];
	
	// Load the file name
	// Make sure BYTE filename[] above is large enough for your longest name
	MPFSGetFilename(curHTTP.file, filename, 20);
	
	// If its the forms.htm page
	if(!memcmppgm2ram(filename, "forms.htm", 9))
	{
		// Seek out each of the four LED strings, and if it exists set the LED states
		ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE *)"led4");
		if(ptr)
			LED4_IO = (*ptr == '1');

		ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE *)"led3");
		if(ptr)
			LED3_IO = (*ptr == '1');

		ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE *)"led2");
		if(ptr)
			LED2_IO = (*ptr == '1');

		ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE *)"led1");
		if(ptr)
			LED1_IO = (*ptr == '1');
	}
	
	// If it's the LED updater file
	else if(!memcmppgm2ram(filename, "cookies.htm", 11))
	{
		// This is very simple.  The names and values we want are already in
		// the data array.  We just set the hasArgs value to indicate how many
		// name/value pairs we want stored as cookies.
		// To add the second cookie, just increment this value.
		// remember to also add a dynamic variable callback to control the printout.
		curHTTP.hasArgs = 0x01;
	}
		
	// If it's the LED updater file
	else if(!memcmppgm2ram(filename, "leds.cgi", 8))
	{
		// Determine which LED to toggle
		ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE *)"led");
		
		// Toggle the specified LED
		switch(*ptr) {
			case '1':
				LED1_IO ^= 1;
				break;
			case '2':
				LED2_IO ^= 1;
				break;
			case '3':
				LED3_IO ^= 1;
				break;
			case '4':
				LED4_IO ^= 1;
				break;
			case '5':
				LED5_IO ^= 1;
				break;
			case '6':
				LED6_IO ^= 1;
				break;
			case '7':
				LED7_IO ^= 1;
				break;
		}
	}
    
    else if(!memcmppgm2ram(filename, "manualcontroller.htm", 20))
    {
        ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE *)"function");
        
        if(*ptr){
            if(strcmppgm2ram((char*)ptr,(ROM char*)"stop") == 0) {
                cvs->state->omegaref[R_ID] = 0.0;
                cvs->state->omegaref[L_ID] = 0.0;
                #ifdef ROBINSUN
                    cvs->outputs->command_fish_vertical = 0.0;
                    cvs->outputs->command_fish_horizontal = 0.0;
                    cvs->outputs->command_blocks = 0.0;
                    MyCyclone_Write(A_PICtoFPGA, 0x0001 & 1);
                #endif          
            }
            else if(strcmppgm2ram((char*)ptr,(ROM char*)"open") == 0){
                cvs->outputs->command_blocks = -25;
            }
            else if(strcmppgm2ram((char*)ptr,(ROM char*)"close") == 0){
                cvs->outputs->command_blocks = 50;
            }
            else if(strcmppgm2ram((char*)ptr,(ROM char*)"fishup") == 0){
                #ifdef ROBINSUN
                    cvs->outputs->command_fish_vertical = -60;
                #endif
            }
            else if(strcmppgm2ram((char*)ptr,(ROM char*)"fishdown") == 0){
                #ifdef ROBINSUN
                    cvs->outputs->command_fish_vertical = 15;
                #endif
            }
            else if(strcmppgm2ram((char*)ptr,(ROM char*)"fishin") == 0){
                #ifdef ROBINSUN
                    cvs->outputs->command_fish_horizontal = -55;
                #endif
            }
            else if(strcmppgm2ram((char*)ptr,(ROM char*)"fishout") == 0){
                #ifdef ROBINSUN
                    cvs->outputs->command_fish_horizontal = 40;
                #endif
            }
        }
    }
	
	return HTTP_IO_DONE;
}


/****************************************************************************
  Section:
	POST Form Handlers
  ***************************************************************************/
#if defined(HTTP_USE_POST)

/*****************************************************************************
  Function:
	HTTP_IO_RESULT HTTPExecutePost(void)
	
  Internal:
  	See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/
HTTP_IO_RESULT HTTPExecutePost(void)
{
	// Resolve which function to use and pass along
	BYTE filename[20];
	
	// Load the file name
	// Make sure BYTE filename[] above is large enough for your longest name
	MPFSGetFilename(curHTTP.file, filename, sizeof(filename));
	
    if(!memcmppgm2ram(filename, "autocontroller.htm", 18))
        return HTTPautocontrol();
    
#if defined(USE_LCD)
	if(!memcmppgm2ram(filename, "forms.htm", 9))
		return HTTPPostLCD();
#endif

#if defined(STACK_USE_HTTP_MD5_DEMO)
	if(!memcmppgm2ram(filename, "upload.htm", 10))
		return HTTPPostMD5();
#endif

#if defined(STACK_USE_HTTP_APP_RECONFIG)
	if(!memcmppgm2ram(filename, "protect/config.htm", 18))
		return HTTPPostConfig();
	#if defined(STACK_USE_SNMP_SERVER)
	else if(!memcmppgm2ram(filename, "snmp/snmpconfig.htm", 19))
		return HTTPPostSNMPCommunity();
	#endif
#endif
	
#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	if(!strcmppgm2ram((char*)filename, "dyndns/index.htm"))
		return HTTPPostDDNSConfig();
#endif

	return HTTP_IO_DONE;
}

/*****************************************************************************
  Function:
	static HTTP_IO_RESULT HTTPPostLCD(void)

  Summary:
	Processes the LCD form on forms.htm

  Description:
	Locates the 'lcd' parameter and uses it to update the text displayed
	on the board's LCD display.
	
	This function has four states.  The first reads a name from the data
	string returned as part of the POST request.  If a name cannot
	be found, it returns, asking for more data.  Otherwise, if the name 
	is expected, it reads the associated value and writes it to the LCD.  
	If the name is not expected, the value is discarded and the next name 
	parameter is read.
	
	In the case where the expected string is never found, this function 
	will eventually return HTTP_IO_NEED_DATA when no data is left.  In that
	case, the HTTP2 server will automatically trap the error and issue an
	Internal Server Error to the browser.

  Precondition:
	None

  Parameters:
	None

  Return Values:
  	HTTP_IO_DONE - the parameter has been found and saved
  	HTTP_IO_WAITING - the function is pausing to continue later
  	HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(USE_LCD)
static HTTP_IO_RESULT HTTPPostLCD(void)
{
	BYTE* cDest;
	
	#define SM_POST_LCD_READ_NAME		(0u)
	#define SM_POST_LCD_READ_VALUE		(1u)
	
	switch(curHTTP.smPost)
	{
		// Find the name
		case SM_POST_LCD_READ_NAME:
		
			// Read a name
			if(HTTPReadPostName(curHTTP.data, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;

			curHTTP.smPost = SM_POST_LCD_READ_VALUE;
			// No break...continue reading value
		
		// Found the value, so store the LCD and return
		case SM_POST_LCD_READ_VALUE:
					
			// If value is expected, read it to data buffer,
			// otherwise ignore it (by reading to NULL)	
			if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"lcd"))
				cDest = curHTTP.data;
			else
				cDest = NULL;
			
			// Read a value string
			if(HTTPReadPostValue(cDest, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;
			
			// If this was an unexpected value, look for a new name
			if(!cDest)
			{
				curHTTP.smPost = SM_POST_LCD_READ_NAME;
				break;
			}
			
			// Copy up to 32 characters to the LCD
			if(strlen((char*)cDest) < 32u)
			{
				memset(LCDText, ' ', 32);
				strcpy((char*)LCDText, (char*)cDest);
			}
			else
			{
				memcpy(LCDText, (void *)cDest, 32);
			}
			LCDUpdate();
			
			// This is the only expected value, so callback is done
			strcpypgm2ram((char*)curHTTP.data, "/forms.htm");
			curHTTP.httpStatus = HTTP_REDIRECT;
			return HTTP_IO_DONE;
	}
	
	// Default assumes that we're returning for state machine convenience.
	// Function will be called again later.
	return HTTP_IO_WAITING;
}
#endif

/*****************************************************************************
  Function:
	static HTTP_IO_RESULT HTTPPostConfig(void)

  Summary:
	Processes the configuration form on config/index.htm

  Description:
	Accepts configuration parameters from the form, saves them to a
	temporary location in RAM, then eventually saves the data to EEPROM or
	external Flash.
	
	When complete, this function redirects to config/reboot.htm, which will
	display information on reconnecting to the board.

	This function creates a shadow copy of the AppConfig structure in 
	RAM and then overwrites incoming data there as it arrives.  For each 
	name/value pair, the name is first read to curHTTP.data[0:5].  Next, the 
	value is read to newAppConfig.  Once all data has been read, the new
	AppConfig is saved back to EEPROM and the browser is redirected to 
	reboot.htm.  That file includes an AJAX call to reboot.cgi, which 
	performs the actual reboot of the machine.
	
	If an IP address cannot be parsed, too much data is POSTed, or any other 
	parsing error occurs, the browser reloads config.htm and displays an error 
	message at the top.

  Precondition:
	None

  Parameters:
	None

  Return Values:
  	HTTP_IO_DONE - all parameters have been processed
  	HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(STACK_USE_HTTP_APP_RECONFIG)
static HTTP_IO_RESULT HTTPPostConfig(void)
{
	APP_CONFIG newAppConfig;
	BYTE *ptr;
	BYTE i;

	// Check to see if the browser is attempting to submit more data than we 
	// can parse at once.  This function needs to receive all updated 
	// parameters and validate them all before committing them to memory so that
	// orphaned configuration parameters do not get written (for example, if a 
	// static IP address is given, but the subnet mask fails parsing, we 
	// should not use the static IP address).  Everything needs to be processed 
	// in a single transaction.  If this is impossible, fail and notify the user.
	// As a web devloper, if you add parameters to AppConfig and run into this 
	// problem, you could fix this by to splitting your update web page into two 
	// seperate web pages (causing two transactional writes).  Alternatively, 
	// you could fix it by storing a static shadow copy of AppConfig someplace 
	// in memory and using it instead of newAppConfig.  Lastly, you could 
	// increase the TCP RX FIFO size for the HTTP server.  This will allow more 
	// data to be POSTed by the web browser before hitting this limit.
	if(curHTTP.byteCount > TCPIsGetReady(sktHTTP) + TCPGetRxFIFOFree(sktHTTP))
		goto ConfigFailure;
	
	// Ensure that all data is waiting to be parsed.  If not, keep waiting for 
	// all of it to arrive.
	if(TCPIsGetReady(sktHTTP) < curHTTP.byteCount)
		return HTTP_IO_NEED_DATA;
	
	
	// Use current config in non-volatile memory as defaults
	#if defined(EEPROM_CS_TRIS)
		XEEReadArray(sizeof(NVM_VALIDATION_STRUCT), (BYTE*)&newAppConfig, sizeof(newAppConfig));
	#elif defined(SPIFLASH_CS_TRIS)
		SPIFlashReadArray(sizeof(NVM_VALIDATION_STRUCT), (BYTE*)&newAppConfig, sizeof(newAppConfig));
	#endif
	
	// Start out assuming that DHCP is disabled.  This is necessary since the 
	// browser doesn't submit this field if it is unchecked (meaning zero).  
	// However, if it is checked, this will be overridden since it will be 
	// submitted.
	newAppConfig.Flags.bIsDHCPEnabled = 0;


	// Read all browser POST data
	while(curHTTP.byteCount)
	{
		// Read a form field name
		if(HTTPReadPostName(curHTTP.data, 6) != HTTP_READ_OK)
			goto ConfigFailure;
			
		// Read a form field value
		if(HTTPReadPostValue(curHTTP.data + 6, sizeof(curHTTP.data)-6-2) != HTTP_READ_OK)
			goto ConfigFailure;
			
		// Parse the value that was read
		if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"ip"))
		{// Read new static IP Address
			if(!StringToIPAddress(curHTTP.data+6, &newAppConfig.MyIPAddr))
				goto ConfigFailure;
				
			newAppConfig.DefaultIPAddr.Val = newAppConfig.MyIPAddr.Val;
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"gw"))
		{// Read new gateway address
			if(!StringToIPAddress(curHTTP.data+6, &newAppConfig.MyGateway))
				goto ConfigFailure;
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"sub"))
		{// Read new static subnet
			if(!StringToIPAddress(curHTTP.data+6, &newAppConfig.MyMask))
				goto ConfigFailure;

			newAppConfig.DefaultMask.Val = newAppConfig.MyMask.Val;
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"dns1"))
		{// Read new primary DNS server
			if(!StringToIPAddress(curHTTP.data+6, &newAppConfig.PrimaryDNSServer))
				goto ConfigFailure;
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"dns2"))
		{// Read new secondary DNS server
			if(!StringToIPAddress(curHTTP.data+6, &newAppConfig.SecondaryDNSServer))
				goto ConfigFailure;
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"mac"))
		{
			// Read new MAC address
			WORD w;
			BYTE i;

			ptr = curHTTP.data+6;

			for(i = 0; i < 12u; i++)
			{// Read the MAC address
				
				// Skip non-hex bytes
				while( *ptr != 0x00u && !(*ptr >= '0' && *ptr <= '9') && !(*ptr >= 'A' && *ptr <= 'F') && !(*ptr >= 'a' && *ptr <= 'f') )
					ptr++;

				// MAC string is over, so zeroize the rest
				if(*ptr == 0x00u)
				{
					for(; i < 12u; i++)
						curHTTP.data[i] = '0';
					break;
				}
				
				// Save the MAC byte
				curHTTP.data[i] = *ptr++;
			}
			
			// Read MAC Address, one byte at a time
			for(i = 0; i < 6u; i++)
			{
				((BYTE*)&w)[1] = curHTTP.data[i*2];
				((BYTE*)&w)[0] = curHTTP.data[i*2+1];
				newAppConfig.MyMACAddr.v[i] = hexatob(*((WORD_VAL*)&w));
			}
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"host"))
		{// Read new hostname
			FormatNetBIOSName(&curHTTP.data[6]);
			memcpy((void*)newAppConfig.NetBIOSName, (void*)curHTTP.data+6, 16);
		}
		else if(!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"dhcp"))
		{// Read new DHCP Enabled flag
			if(curHTTP.data[6] == '1')
				newAppConfig.Flags.bIsDHCPEnabled = 1;
		}
	}


	// All parsing complete!  Save new settings and force a reboot
	MyWIFI_SaveAppConfig(&newAppConfig);
	
	// Set the board to reboot and display reconnecting information
	strcpypgm2ram((char*)curHTTP.data, "/protect/reboot.htm?");
	memcpy((void*)(curHTTP.data+20), (void*)newAppConfig.NetBIOSName, 16);
	curHTTP.data[20+16] = 0x00;	// Force null termination
	for(i = 20; i < 20u+16u; i++)
	{
		if(curHTTP.data[i] == ' ')
			curHTTP.data[i] = 0x00;
	}		
	curHTTP.httpStatus = HTTP_REDIRECT;	
	
	return HTTP_IO_DONE;


ConfigFailure:
	lastFailure = TRUE;
	strcpypgm2ram((char*)curHTTP.data, "/protect/config.htm");
	curHTTP.httpStatus = HTTP_REDIRECT;		

	return HTTP_IO_DONE;
}

#if defined(STACK_USE_SNMP_SERVER)
static HTTP_IO_RESULT HTTPPostSNMPCommunity(void)
{
	BYTE vCommunityIndex;
	BYTE *dest;

	#define SM_CFG_SNMP_READ_NAME	(0u)
	#define SM_CFG_SNMP_READ_VALUE	(1u)
	
	switch(curHTTP.smPost)
	{
		case SM_CFG_SNMP_READ_NAME:
			// If all parameters have been read, end
			if(curHTTP.byteCount == 0u)
			{
				MyWIFI_SaveAppConfig(&AppConfig);
				return HTTP_IO_DONE;
			}
		
			// Read a name
			if(HTTPReadPostName(curHTTP.data, sizeof(curHTTP.data)-2) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;
				
			// Move to reading a value, but no break
			curHTTP.smPost = SM_CFG_SNMP_READ_VALUE;
			
		case SM_CFG_SNMP_READ_VALUE:
			// Read a value
			if(HTTPReadPostValue(curHTTP.data + 6, sizeof(curHTTP.data)-6-2) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;

			// Default action after this is to read the next name, unless there's an error
			curHTTP.smPost = SM_CFG_SNMP_READ_NAME;

			// See if this is a known parameter and legal (must be null 
			// terminator in 4th field name byte, string must no greater than 
			// SNMP_COMMUNITY_MAX_LEN bytes long, and SNMP_MAX_COMMUNITY_SUPPORT 
			// must not be violated.
			vCommunityIndex = curHTTP.data[3] - '0';
			if(vCommunityIndex >= SNMP_MAX_COMMUNITY_SUPPORT)
				break;
			if(curHTTP.data[4] != 0x00u)
				break;
			if(memcmppgm2ram((void*)curHTTP.data, (ROM void*)"rcm", 3) == 0)
				dest = AppConfig.readCommunity[vCommunityIndex];
			else if(memcmppgm2ram((void*)curHTTP.data, (ROM void*)"wcm", 3) == 0)
				dest = AppConfig.writeCommunity[vCommunityIndex];
			else
				break;
			if(strlen((char*)curHTTP.data + 6) > SNMP_COMMUNITY_MAX_LEN)
				break;
			
			// String seems valid, lets copy it to AppConfig
			strcpy((char*)dest, (char*)curHTTP.data+6);
			break;			
	}

	return HTTP_IO_WAITING;		// Assume we're waiting to process more data
}
#endif //#if defined(STACK_USE_SNMP_SERVER)

#endif	// #if defined(STACK_USE_HTTP_APP_RECONFIG)

/*****************************************************************************
  Function:
	static HTTP_IO_RESULT HTTPPostMD5(void)

  Summary:
	Processes the file upload form on upload.htm

  Description:
	This function demonstrates the processing of file uploads.  First, the
	function locates the file data, skipping over any headers that arrive.
	Second, it reads the file 64 bytes at a time and hashes that data.  Once
	all data has been received, the function calculates the MD5 sum and
	stores it in curHTTP.data.

	After the headers, the first line from the form will be the MIME 
	separator.  Following that is more headers about the file, which we 
	discard.  After another CRLFCRLF, the file data begins, and we read 
	it 16 bytes at a time and add that to the MD5 calculation.  The reading
	terminates when the separator string is encountered again on its own 
	line.  Notice that the actual file data is trashed in this process, 
	allowing us to accept files of arbitrary size, not limited by RAM.  
	Also notice that the data buffer is used as an arbitrary storage array 
	for the result.  The ~uploadedmd5~ callback reads this data later to 
	send back to the client.
	
  Precondition:
	None

  Parameters:
	None

  Return Values:
	HTTP_IO_DONE - all parameters have been processed
	HTTP_IO_WAITING - the function is pausing to continue later
	HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/
#if defined(STACK_USE_HTTP_MD5_DEMO)
static HTTP_IO_RESULT HTTPPostMD5(void)
{
	WORD lenA, lenB;
	static HASH_SUM md5;			// Assume only one simultaneous MD5
	
	#define SM_MD5_READ_SEPARATOR	(0u)
	#define SM_MD5_SKIP_TO_DATA		(1u)
	#define SM_MD5_READ_DATA		(2u)
	#define SM_MD5_POST_COMPLETE	(3u)
	
	// Don't care about curHTTP.data at this point, so use that for buffer
	switch(curHTTP.smPost)
	{
		// Just started, so try to find the separator string
		case SM_MD5_READ_SEPARATOR:
			// Reset the MD5 calculation
			MD5Initialize(&md5);
			
			// See if a CRLF is in the buffer
			lenA = TCPFindROMArray(sktHTTP, (ROM BYTE*)"\r\n", 2, 0, FALSE);
			if(lenA == 0xffff)
			{//if not, ask for more data
				return HTTP_IO_NEED_DATA;
			}
		
			// If so, figure out where the last byte of data is
			// Data ends at CRLFseparator--CRLF, so 6+len bytes
			curHTTP.byteCount -= lenA + 6;
			
			// Read past the CRLF
			curHTTP.byteCount -= TCPGetArray(sktHTTP, NULL, lenA+2);
			
			// Save the next state (skip to CRLFCRLF)
			curHTTP.smPost = SM_MD5_SKIP_TO_DATA;
			
			// No break...continue reading the headers if possible
				
		// Skip the headers
		case SM_MD5_SKIP_TO_DATA:
			// Look for the CRLFCRLF
			lenA = TCPFindROMArray(sktHTTP, (ROM BYTE*)"\r\n\r\n", 4, 0, FALSE);
	
			if(lenA != 0xffff)
			{// Found it, so remove all data up to and including
				lenA = TCPGetArray(sktHTTP, NULL, lenA+4);
				curHTTP.byteCount -= lenA;
				curHTTP.smPost = SM_MD5_READ_DATA;
			}
			else
			{// Otherwise, remove as much as possible
				lenA = TCPGetArray(sktHTTP, NULL, TCPIsGetReady(sktHTTP) - 4);
				curHTTP.byteCount -= lenA;
			
				// Return the need more data flag
				return HTTP_IO_NEED_DATA;
			}
			
			// No break if we found the header terminator
			
		// Read and hash file data
		case SM_MD5_READ_DATA:
			// Find out how many bytes are available to be read
			lenA = TCPIsGetReady(sktHTTP);
			if(lenA > curHTTP.byteCount)
				lenA = curHTTP.byteCount;
	
			while(lenA > 0u)
			{// Add up to 64 bytes at a time to the sum
				lenB = TCPGetArray(sktHTTP, curHTTP.data, (lenA < 64u)?lenA:64);
				curHTTP.byteCount -= lenB;
				lenA -= lenB;
				MD5AddData(&md5, curHTTP.data, lenB);
			}
					
			// If we've read all the data
			if(curHTTP.byteCount == 0u)
			{// Calculate and copy result to curHTTP.data for printout
				curHTTP.smPost = SM_MD5_POST_COMPLETE;
				MD5Calculate(&md5, curHTTP.data);
				return HTTP_IO_DONE;
			}
				
			// Ask for more data
			return HTTP_IO_NEED_DATA;
	}
	
	return HTTP_IO_DONE;
}
#endif // #if defined(STACK_USE_HTTP_MD5_DEMO)


/****************************************************************************
  Function:
    HTTP_IO_RESULT HTTPPostDDNSConfig(void)
    
  Summary:
    Parsing and collecting http data received from http form.

  Description:
    This routine will be excuted every time the Dynamic DNS Client
    configuration form is submitted.  The http data is received 
    as a string of the variables seperated by '&' characters in the TCP RX
    buffer.  This data is parsed to read the required configuration values, 
    and those values are populated to the global array (DDNSData) reserved 
    for this purpose.  As the data is read, DDNSPointers is also populated
    so that the dynamic DNS client can execute with the new parameters.
    
  Precondition:
     curHTTP is loaded.

  Parameters:
    None.

  Return Values:
    HTTP_IO_DONE 		-  Finished with procedure
    HTTP_IO_NEED_DATA	-  More data needed to continue, call again later
    HTTP_IO_WAITING 	-  Waiting for asynchronous process to complete, 
    						call again later
  ***************************************************************************/
#if defined(STACK_USE_DYNAMICDNS_CLIENT)
static HTTP_IO_RESULT HTTPPostDDNSConfig(void)
{
	static BYTE *ptrDDNS;

	#define SM_DDNS_START			(0u)
	#define SM_DDNS_READ_NAME		(1u)
	#define SM_DDNS_READ_VALUE		(2u)
	#define SM_DDNS_READ_SERVICE	(3u)
	#define SM_DDNS_DONE			(4u)

	#define DDNS_SPACE_REMAINING				(sizeof(DDNSData) - (ptrDDNS - DDNSData))

	switch(curHTTP.smPost)
	{
		// Sets defaults for the system
		case SM_DDNS_START:
			ptrDDNS = DDNSData;
			DDNSSetService(0);
			DDNSClient.Host.szROM = NULL;
			DDNSClient.Username.szROM = NULL;
			DDNSClient.Password.szROM = NULL;
			DDNSClient.ROMPointers.Host = 0;
			DDNSClient.ROMPointers.Username = 0;
			DDNSClient.ROMPointers.Password = 0;
			curHTTP.smPost++;
			
		// Searches out names and handles them as they arrive
		case SM_DDNS_READ_NAME:
			// If all parameters have been read, end
			if(curHTTP.byteCount == 0u)
			{
				curHTTP.smPost = SM_DDNS_DONE;
				break;
			}
		
			// Read a name
			if(HTTPReadPostName(curHTTP.data, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;
			
			if(!strcmppgm2ram((char *)curHTTP.data, (ROM char*)"service"))
			{
				// Reading the service (numeric)
				curHTTP.smPost = SM_DDNS_READ_SERVICE;
				break;
			}
			else if(!strcmppgm2ram((char *)curHTTP.data, (ROM char*)"user"))
				DDNSClient.Username.szRAM = ptrDDNS;
			else if(!strcmppgm2ram((char *)curHTTP.data, (ROM char*)"pass"))
				DDNSClient.Password.szRAM = ptrDDNS;
			else if(!strcmppgm2ram((char *)curHTTP.data, (ROM char*)"host"))
				DDNSClient.Host.szRAM = ptrDDNS;
			
			// Move to reading the value for user/pass/host
			curHTTP.smPost++;
			
		// Reads in values and assigns them to the DDNS RAM
		case SM_DDNS_READ_VALUE:
			// Read a name
			if(HTTPReadPostValue(ptrDDNS, DDNS_SPACE_REMAINING) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;
				
			// Move past the data that was just read
			ptrDDNS += strlen((char*)ptrDDNS);
			if(ptrDDNS < DDNSData + sizeof(DDNSData) - 1)
				ptrDDNS += 1;			
			
			// Return to reading names
			curHTTP.smPost = SM_DDNS_READ_NAME;
			break;
		
		// Reads in a service ID
		case SM_DDNS_READ_SERVICE:
			// Read the integer id
			if(HTTPReadPostValue(curHTTP.data, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
				return HTTP_IO_NEED_DATA;
			
			// Convert to a service ID
			DDNSSetService((BYTE)atol((char*)curHTTP.data));

			// Return to reading names
			curHTTP.smPost = SM_DDNS_READ_NAME;
			break;
			
		// Sets up the DDNS client for an update
		case SM_DDNS_DONE:
			// Since user name and password changed, force an update immediately
			DDNSForceUpdate();
			
			// Redirect to prevent POST errors
			lastSuccess = TRUE;
			strcpypgm2ram((char*)curHTTP.data, "/dyndns/index.htm");
			curHTTP.httpStatus = HTTP_REDIRECT;
			return HTTP_IO_DONE;				
	}
	
	return HTTP_IO_WAITING;		// Assume we're waiting to process more data
}
#endif	// #if defined(STACK_USE_DYNAMICDNS_CLIENT)

#endif //(use_post)


/****************************************************************************
  Section:
	Dynamic Variable Callback Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void HTTPPrint_varname(void)
	
  Internal:
  	See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/

void HTTPPrint_builddate(void)
{
    curHTTP.callbackPos = 0x01;
    if(TCPIsPutReady(sktHTTP) < strlenpgm((ROM char*)__DATE__" ""00:08:00"))
        return;
 
    curHTTP.callbackPos = 0x00;
    TCPPutROMString(sktHTTP, (ROM void*)__DATE__" ""00:08:00");
}

void HTTPPrint_version(void)
{
	TCPPutROMString(sktHTTP, (ROM void*)TCPIP_STACK_VERSION);
}




ROM BYTE HTML_UP_ARROW[] = "up";
ROM BYTE HTML_DOWN_ARROW[] = "dn";

void HTTPPrint_btn(WORD num)
{
	// Determine which button
	switch(num)
	{
		case 0:
			num = MyCyclone_Switch & 0x1;
			break;
		case 1:
			num = MyCyclone_Switch & 0x2;
			break;
		case 2:
			num = MyCyclone_Switch & 0x4;
			break;
		case 3:
			num = MyCyclone_Switch & 0x8;
			break;
		default:
			num = 0;
	}

	// Print the output
	TCPPutROMString(sktHTTP, (num?HTML_UP_ARROW:HTML_DOWN_ARROW));
	return;
}
	
void HTTPPrint_led(WORD num)
{
	// Determine which LED
	switch(num)
	{
		case 0:
			num = MyCyclone_LED & 0x1;
			break;
		case 1:
			num = MyCyclone_LED & 0x2;
			break;
		case 2:
			num = MyCyclone_LED & 0x4;
			break;
		case 3:
			num = MyCyclone_LED & 0x8;
			break;
		case 4:
			num = MyCyclone_LED & 0x10;
			break;
		case 5:
			num = MyCyclone_LED & 0x20;
			break;
		case 6:
			num = MyCyclone_LED & 0x40;
			break;
		case 7:
			num = MyCyclone_LED & 0x80;
			break;

		default:
			num = 0;
	}

	// Print the output
	TCPPut(sktHTTP, (num?'1':'0'));
	return;
}

void HTTPPrint_ledSelected(WORD num, WORD state)
{
	// Determine which LED to check
	switch(num)
	{
		case 0:
			num = LED0_IO;
			break;
		case 1:
			num = LED1_IO;
			break;
		case 2:
			num = LED2_IO;
			break;
		case 3:
			num = LED3_IO;
			break;
		case 4:
			num = LED4_IO;
			break;
		case 5:
			num = LED5_IO;
			break;
		case 6:
			num = LED6_IO;
			break;
		case 7:
			num = LED7_IO;
			break;

		default:
			num = 0;
	}
	
	// Print output if TRUE and ON or if FALSE and OFF
	if((state && num) || (!state && !num))
		TCPPutROMString(sktHTTP, (ROM BYTE*)"SELECTED");
	return;
}

void HTTPPrint_pot(void)
{
    // Modified for MyPicCycloneNano : Display Temperature instead of Potentiometer
    BYTE theStr[64];
        WORD  theTemperature;

        theTemperature = MyTemperature_Read();
        if (theTemperature >= 0x80)
            theTemperature |= 0xffffff00;   // Sign Extend
        sprintf(theStr, "%d°", theTemperature);

   	TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_lcdtext(void)
{
	WORD len;

	// Determine how many bytes we can write
	len = TCPIsPutReady(sktHTTP);
	
	#if defined(USE_LCD)
	// If just starting, set callbackPos
	if(curHTTP.callbackPos == 0u)
		curHTTP.callbackPos = 32;
	
	// Write a byte at a time while we still can
	// It may take up to 12 bytes to write a character
	// (spaces and newlines are longer)
	while(len > 12u && curHTTP.callbackPos)
	{
		// After 16 bytes write a newline
		if(curHTTP.callbackPos == 16u)
			len -= TCPPutROMArray(sktHTTP, (ROM BYTE*)"<br />", 6);

		if(LCDText[32-curHTTP.callbackPos] == ' ' || LCDText[32-curHTTP.callbackPos] == '\0')
			len -= TCPPutROMArray(sktHTTP, (ROM BYTE*)"&nbsp;", 6);
		else
			len -= TCPPut(sktHTTP, LCDText[32-curHTTP.callbackPos]);

		curHTTP.callbackPos--;
	}
	#else
	TCPPutROMString(sktHTTP, (ROM BYTE*)"No LCD Present");
	#endif

	return;
}

void HTTPPrint_cookiename(void)
{
	BYTE *ptr;
	
	ptr = HTTPGetROMArg(curHTTP.data, (ROM BYTE*)"name");
	
	if(ptr)
		TCPPutString(sktHTTP, ptr);
	else
		TCPPutROMString(sktHTTP, (ROM BYTE*)"not set");
	
	return;
}

void HTTPPrint_uploadedmd5(void)
{
	BYTE i;

	// Set a flag to indicate not finished
	curHTTP.callbackPos = 1;
	
	// Make sure there's enough output space
	if(TCPIsPutReady(sktHTTP) < 32u + 37u + 5u)
		return;

	// Check for flag set in HTTPPostMD5
#if defined(STACK_USE_HTTP_MD5_DEMO)
	if(curHTTP.smPost != SM_MD5_POST_COMPLETE)
#endif
	{// No file uploaded, so just return
		TCPPutROMString(sktHTTP, (ROM BYTE*)"<b>Upload a File</b>");
		curHTTP.callbackPos = 0;
		return;
	}
	
	TCPPutROMString(sktHTTP, (ROM BYTE*)"<b>Uploaded File's MD5 was:</b><br />");
	
	// Write a byte of the md5 sum at a time
	for(i = 0; i < 16u; i++)
	{
		TCPPut(sktHTTP, btohexa_high(curHTTP.data[i]));
		TCPPut(sktHTTP, btohexa_low(curHTTP.data[i]));
		if((i & 0x03) == 3u)
			TCPPut(sktHTTP, ' ');
	}
	
	curHTTP.callbackPos = 0x00;
	return;
}

extern APP_CONFIG AppConfig;

void HTTPPrintIP(IP_ADDR ip)
{
	BYTE digits[4];
	BYTE i;
	
	for(i = 0; i < 4u; i++)
	{
		if(i)
			TCPPut(sktHTTP, '.');
		uitoa(ip.v[i], digits);
		TCPPutString(sktHTTP, digits);
	}
}

void HTTPPrint_config_hostname(void)
{
	TCPPutString(sktHTTP, AppConfig.NetBIOSName);
	return;
}

void HTTPPrint_config_dhcpchecked(void)
{
	if(AppConfig.Flags.bIsDHCPEnabled)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"checked");
	return;
}

void HTTPPrint_config_ip(void)
{
	HTTPPrintIP(AppConfig.MyIPAddr);
	return;
}

void HTTPPrint_config_gw(void)
{
	HTTPPrintIP(AppConfig.MyGateway);
	return;
}

void HTTPPrint_config_subnet(void)
{
	HTTPPrintIP(AppConfig.MyMask);
	return;
}

void HTTPPrint_config_dns1(void)
{
	HTTPPrintIP(AppConfig.PrimaryDNSServer);
	return;
}

void HTTPPrint_config_dns2(void)
{
	HTTPPrintIP(AppConfig.SecondaryDNSServer);
	return;
}

void HTTPPrint_config_mac(void)
{
	BYTE i;
	
	if(TCPIsPutReady(sktHTTP) < 18u)
	{//need 17 bytes to write a MAC
		curHTTP.callbackPos = 0x01;
		return;
	}	
	
	// Write each byte
	for(i = 0; i < 6u; i++)
	{
		if(i)
			TCPPut(sktHTTP, ':');
		TCPPut(sktHTTP, btohexa_high(AppConfig.MyMACAddr.v[i]));
		TCPPut(sktHTTP, btohexa_low(AppConfig.MyMACAddr.v[i]));
	}
	
	// Indicate that we're done
	curHTTP.callbackPos = 0x00;
	return;
}


// SNMP Read communities configuration page
void HTTPPrint_read_comm(WORD num)
{
	#if defined(STACK_USE_SNMP_SERVER)
	// Ensure no one tries to read illegal memory addresses by specifying 
	// illegal num values.
	if(num >= SNMP_MAX_COMMUNITY_SUPPORT)
		return;
		
	// Send proper string
	TCPPutString(sktHTTP, AppConfig.readCommunity[num]);
	#endif
}

// SNMP Write communities configuration page
void HTTPPrint_write_comm(WORD num)
{
	#if defined(STACK_USE_SNMP_SERVER)
	// Ensure no one tries to read illegal memory addresses by specifying 
	// illegal num values.
	if(num >= SNMP_MAX_COMMUNITY_SUPPORT)
		return;
		
	// Send proper string
	TCPPutString(sktHTTP, AppConfig.writeCommunity[num]);
	#endif
}


void HTTPPrint_reboot(void)
{
	// This is not so much a print function, but causes the board to reboot
	// when the configuration is changed.  If called via an AJAX call, this
	// will gracefully reset the board and bring it back online immediately
	Reset();
}

void HTTPPrint_rebootaddr(void)
{// This is the expected address of the board upon rebooting
	TCPPutString(sktHTTP, curHTTP.data);	
}

void HTTPPrint_ddns_user(void)
{
	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	if(DDNSClient.ROMPointers.Username || !DDNSClient.Username.szRAM)
		return;
	if(curHTTP.callbackPos == 0x00u)
		curHTTP.callbackPos = (PTR_BASE)DDNSClient.Username.szRAM;
	curHTTP.callbackPos = (PTR_BASE)TCPPutString(sktHTTP, (BYTE*)(PTR_BASE)curHTTP.callbackPos);
	if(*(BYTE*)(PTR_BASE)curHTTP.callbackPos == '\0')
		curHTTP.callbackPos = 0x00;
	#endif
}

void HTTPPrint_ddns_pass(void)
{
	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	if(DDNSClient.ROMPointers.Password || !DDNSClient.Password.szRAM)
		return;
	if(curHTTP.callbackPos == 0x00u)
		curHTTP.callbackPos = (PTR_BASE)DDNSClient.Password.szRAM;
	curHTTP.callbackPos = (PTR_BASE)TCPPutString(sktHTTP, (BYTE*)(PTR_BASE)curHTTP.callbackPos);
	if(*(BYTE*)(PTR_BASE)curHTTP.callbackPos == '\0')
		curHTTP.callbackPos = 0x00;
	#endif
}

void HTTPPrint_ddns_host(void)
{
	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	if(DDNSClient.ROMPointers.Host || !DDNSClient.Host.szRAM)
		return;
	if(curHTTP.callbackPos == 0x00u)
		curHTTP.callbackPos = (PTR_BASE)DDNSClient.Host.szRAM;
	curHTTP.callbackPos = (PTR_BASE)TCPPutString(sktHTTP, (BYTE*)(PTR_BASE)curHTTP.callbackPos);
	if(*(BYTE*)(PTR_BASE)curHTTP.callbackPos == '\0')
		curHTTP.callbackPos = 0x00;
	#endif
}

extern ROM char * ROM ddnsServiceHosts[];
void HTTPPrint_ddns_service(WORD i)
{
	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	if(!DDNSClient.ROMPointers.UpdateServer || !DDNSClient.UpdateServer.szROM)
		return;
	if((ROM char*)DDNSClient.UpdateServer.szROM == ddnsServiceHosts[i])
		TCPPutROMString(sktHTTP, (ROM BYTE*)"selected");
	#endif
}


void HTTPPrint_ddns_status(void)
{
	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	DDNS_STATUS s;
	s = DDNSGetLastStatus();
	if(s == DDNS_STATUS_GOOD || s == DDNS_STATUS_UNCHANGED || s == DDNS_STATUS_NOCHG)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"ok");
	else if(s == DDNS_STATUS_UNKNOWN)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"unk");
	else
		TCPPutROMString(sktHTTP, (ROM BYTE*)"fail");
	#else
	TCPPutROMString(sktHTTP, (ROM BYTE*)"fail");
	#endif
}

void HTTPPrint_ddns_status_msg(void)
{
	if(TCPIsPutReady(sktHTTP) < 75u)
	{
		curHTTP.callbackPos = 0x01;
		return;
	}

	#if defined(STACK_USE_DYNAMICDNS_CLIENT)
	switch(DDNSGetLastStatus())
	{
		case DDNS_STATUS_GOOD:
		case DDNS_STATUS_NOCHG:
			TCPPutROMString(sktHTTP, (ROM BYTE*)"The last update was successful.");
			break;
		case DDNS_STATUS_UNCHANGED:
			TCPPutROMString(sktHTTP, (ROM BYTE*)"The IP has not changed since the last update.");
			break;
		case DDNS_STATUS_UPDATE_ERROR:
		case DDNS_STATUS_CHECKIP_ERROR:
			TCPPutROMString(sktHTTP, (ROM BYTE*)"Could not communicate with DDNS server.");
			break;
		case DDNS_STATUS_INVALID:
			TCPPutROMString(sktHTTP, (ROM BYTE*)"The current configuration is not valid.");
			break;
		case DDNS_STATUS_UNKNOWN:
			TCPPutROMString(sktHTTP, (ROM BYTE*)"The Dynamic DNS client is pending an update.");
			break;
		default:
			TCPPutROMString(sktHTTP, (ROM BYTE*)"An error occurred during the update.<br />The DDNS Client is suspended.");
			break;
	}
	#else
	TCPPutROMString(sktHTTP, (ROM BYTE*)"The Dynamic DNS Client is not enabled.");
	#endif
	
	curHTTP.callbackPos = 0x00;
}

static HTTP_IO_RESULT HTTPautocontrol(void) 
{
    BYTE *ptr = curHTTP.data;
    int formInput = 0;

    switch (curHTTP.smPost) {
        case POST_READ:

            if (curHTTP.byteCount == 0u) {
                curHTTP.smPost = POST_DONE;
                break;
            }

            if (HTTPReadPostName(ptr, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            if (!strcmppgm2ram((char *) ptr, (ROM char*) "kileft"))
                formInput = 1;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kpleft"))
                formInput = 2;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kiright"))
                formInput = 3;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kpright"))
                formInput = 4;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "refspeed"))
                formInput = 5;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kdright"))
                formInput = 6;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kdleft"))
                formInput = 7;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "linsatv"))
                formInput = 8;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "linsatw"))
                formInput = 9;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "angsatv"))
                formInput = 10;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "angsatw"))
                formInput = 11;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kpangv"))
                formInput = 12;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kpangw"))
                formInput = 13;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kdangv"))
                formInput = 14;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kdangw"))
                formInput = 15;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "kplin"))
                formInput = 16;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "distmin"))
                formInput = 17;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "minspeed"))
                formInput = 18;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "anglev"))
                formInput = 19;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "refangle"))
                formInput = 20;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "x"))
                formInput = 21;
            else if (!strcmppgm2ram((char *) ptr, (ROM char*) "y"))
                formInput = 22;
            
            if (HTTPReadPostValue(ptr, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
                return HTTP_IO_NEED_DATA;

            if (formInput == 1) {
                cvs->param->Ki[L_ID] = atof(ptr);
            } else if (formInput == 2) {
                cvs->param->Kp[L_ID] = atof(ptr);
            } else if (formInput == 3) {
                cvs->param->Ki[R_ID] = atof(ptr);
            } else if (formInput == 4) {
                cvs->param->Kp[R_ID] = atof(ptr);
            } else if (formInput == 5) {
                cvs->param->refspeed = atof(ptr);
            } else if (formInput == 6) {
                cvs->param->Kd[R_ID] = atof(ptr);
            } else if (formInput == 7) {
                cvs->param->Kd[L_ID] = atof(ptr);
            } else if (formInput == 8) {
                cvs->param->linsatv = atof(ptr);
            } else if (formInput == 9) {
                cvs->param->linsatw = atof(ptr);
            } else if (formInput == 10) {
                cvs->param->angsatv = atof(ptr);
            } else if (formInput == 11) {
                cvs->param->angsatw = atof(ptr);
            } else if (formInput == 12) {
                cvs->param->kpangv = atof(ptr);
            } else if (formInput == 13) {
                cvs->param->kpangw = atof(ptr);
            } else if (formInput == 14) {
                cvs->param->kdangv = atof(ptr);
            } else if (formInput == 15) {
                cvs->param->kdangw = atof(ptr);
            } else if (formInput == 16) {
                cvs->param->kplin = atof(ptr);
            } else if (formInput == 17) {
                cvs->param->distmin = atof(ptr);
            } else if (formInput == 18) {
                cvs->param->minspeed = atof(ptr);
            } else if (formInput == 19) {
                cvs->param->anglev = atof(ptr);
            } else if (formInput == 20) {
                cvs->param->refangle = atof(ptr);
            } else if (formInput == 21) {
                cvs->param->xref = atof(ptr);
            } else if (formInput == 22) {
                cvs->param->yref = atof(ptr);
            }

            curHTTP.smPost = POST_READ;
            break;
        case POST_DONE:
            cvs->state->errorIntL = 0.0;
            cvs->state->errorIntR = 0.0;

            strcpypgm2ram((char*) ptr, "autocontroller.htm");
            curHTTP.httpStatus = HTTP_REDIRECT;
            return HTTP_IO_DONE;
    }

    return HTTP_IO_WAITING;
}

void HTTPPrint_smtps_en(void)
{
	#if defined(STACK_USE_SSL_CLIENT)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"inline");
	#else
		TCPPutROMString(sktHTTP, (ROM BYTE*)"none");
	#endif
}

void HTTPPrint_snmp_en(void)
{
	#if defined(STACK_USE_SNMP_SERVER)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"none");
	#else
		TCPPutROMString(sktHTTP, (ROM BYTE*)"block");
	#endif
}

void HTTPPrint_status_ok(void)
{
	if(lastSuccess)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"block");
	else
		TCPPutROMString(sktHTTP, (ROM BYTE*)"none");
	lastSuccess = FALSE;
}

void HTTPPrint_status_fail(void)
{
	if(lastFailure)
		TCPPutROMString(sktHTTP, (ROM BYTE*)"block");
	else
		TCPPutROMString(sktHTTP, (ROM BYTE*)"none");
	lastFailure = FALSE;
}

void HTTPPrint_r_wheel_speed(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->inputs->r_wheel_speed*.0325*100);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_l_wheel_speed(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->inputs->l_wheel_speed*0.0325*100);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_buttonL(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%d", cvs->inputs->u_switch[L_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_buttonR(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%d", cvs->inputs->u_switch[R_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_omegaref_R(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->omegaref[R_ID]*.0325*100);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_omegaref_L(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->omegaref[L_ID]*.0325*100);
    TCPPutString(sktHTTP, theStr);
}

#ifdef ROBINSUN
void HTTPPrint_sonar1(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->state->avSonar[0]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_sonar2(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->state->avSonar[1]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_sonar3(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->state->avSonar[2]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_sonar4(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->state->avSonar[3]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_sonar5(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->state->avSonar[4]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_sonar6(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->state->avSonar[5]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_speed_blocks(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->inputs->speed_blocks);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_speed_fish_horizontal(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->inputs->speed_fish_horizontal);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_speed_fish_vertical(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->inputs->speed_fish_vertical);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_command_blocks(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->outputs->command_blocks);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_command_fish_horizontal(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->outputs->command_fish_horizontal);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_command_fish_vertical(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->outputs->command_fish_vertical);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_odometer_R(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->inputs->odo_r_speed*.0223*100);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_odometer_L(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->inputs->odo_l_speed*.0223*100);
    TCPPutString(sktHTTP, theStr);
}
#endif

void HTTPPrint_position0(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->position[0]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_position1(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->position[1]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_position2(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->position[2]*180/M_PI);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_goal0(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->goal_position[0]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_goal1(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->goal_position[1]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_goal2(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->goal_position[2]*180/M_PI);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_astaractive(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%d", cvs->param->ready_start_astar);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_intergoal0(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->intermediate_goal[0]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_intergoal1(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->intermediate_goal[1]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_intergoal2(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->intermediate_goal[2]*180/M_PI);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_competition_state(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%s", getStateName(cvs->state->current_objective));
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_current_progress(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%s", getProgress(cvs->state->current_objective, cvs->state->current_action_progress));
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_time(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->inputs->t);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kpleft(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.4f", cvs->param->Kp[L_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kpright(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.4f", cvs->param->Kp[R_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kileft(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.4f", cvs->param->Ki[L_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kiright(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.4f", cvs->param->Ki[R_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kdleft(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.4f", cvs->param->Kd[L_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kdright(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.4f", cvs->param->Kd[R_ID]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_refspeed(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.2f", cvs->param->refspeed);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_irL(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->inputs->irL);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_irR(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->inputs->irR);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent0(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[0]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent1(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[1]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent2(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[2]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent3(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[3]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent4(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[4]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent5(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[5]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent6(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[6]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponent7(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->state->opponent_position[7]);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_opponentsdetected(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%d", cvs->state->nb_opponents_detected);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_linsatv(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->linsatv);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_linsatw(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->linsatw);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_angsatv(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->angsatv);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_angsatw(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->angsatw);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kpangv(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->kpangv);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kpangw(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->kpangw);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kdangv(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->kdangv);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kdangw(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->kdangw);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_kplin(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->kplin);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_distmin(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->distmin);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_minspeed(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.3f", cvs->param->minspeed);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_anglev(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.1f", cvs->param->anglev);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_refangle(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.1f", cvs->param->refangle);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_x(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.1f", cvs->param->xref);
    TCPPutString(sktHTTP, theStr);
}

void HTTPPrint_y(void)
{
    BYTE theStr[64];
    sprintf(theStr, "%.1f", cvs->param->yref);
    TCPPutString(sktHTTP, theStr);
}

#endif