/**
* Pozyx_core.cpp
* --------------
* This file contains the defintion of the core POZYX functions and variables
*
*/

#include "Pozyx.h"
//#include "MyApp.h"
#include "MyI2C.h"
#define BUFFER_LENGTH 32

#include "Pozyx_definitions.h"

/**
 * The interrupt handler for the pozyx device: keeping it uber short!
 */
void IRQ()
{  
  _interrupt = 1;  
}

BOOL waitForFlag(uint8_t interrupt_flag, int timeout_ms)
{
  long timer = ReadCoreTimer()/(SYS_FREQ/2000.0);
  
  // stay in this loop until the event interrupt flag is set or until the the timer runs out
  while(ReadCoreTimer()/(SYS_FREQ/2000.0)-timer < timeout_ms)
  {
    // in polling mode, we insert a small MyDelayMs such that we don't swamp the i2c bus
    if( _mode == MODE_POLLING ){
      MyDelayMs(1);
    }
    
    if( (_interrupt == 1) || (_mode == MODE_POLLING))
    { 
      _interrupt = 0;
      
      // Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
      uint8_t interrupt_status = 0;
      regRead(POZYX_INT_STATUS, &interrupt_status, 1);
      if((interrupt_status & interrupt_flag) == interrupt_flag)
      {
        // the interrupt we were waiting for arrived!
        return TRUE;
      }
    }     
  } 
  // too bad, pozyx didn't respond 
  // 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
  // 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
  return FALSE;  
}

int begin(BOOL print_result, int mode, int interrupts, int interrupt_pin){
  
  int status = POZYX_SUCCESS;

  if(print_result){
//    Serial.println("Pozyx Shield");
//    Serial.println("------------");
  }

  // check if the mode parameter is valid
  if((mode != MODE_POLLING) && (mode != MODE_INTERRUPT)) 
    return POZYX_FAILURE;
  
  // check if the pin is valid
  if((interrupt_pin != 0) && (interrupt_pin != 1)) 
    return POZYX_FAILURE;
  
  // wait a bit until the pozyx board is up and running
  MyDelayMs(250);
  
  _mode = mode;
  
  uint8_t whoami, selftest;  
  uint8_t regs[3];
  regs[2] = 0x12;
  
  // we read out the first 3 register values: who_am_i, firmware_version and harware version, respectively.
  if(regRead(POZYX_WHO_AM_I, regs, 3) == POZYX_FAILURE){
    return POZYX_FAILURE;
  }  
  whoami = regs[0];
  _sw_version = regs[1];
  _hw_version = regs[2]; 

  if(1){
	  char msg[1024];
	  sprintf(msg, "WhoAmI: %X\n", whoami);
	  MyConsole_SendMsg(msg);
	  sprintf(msg, "SW Version: %d\n", _sw_version);
	  MyConsole_SendMsg(msg);
	  sprintf(msg, "HW Version: %d\n", _hw_version);
	  MyConsole_SendMsg(msg);
      setLedConfig(0x03, NULL);
  }
  // verify if the whoami is correct
  if(whoami != 0x43) {    
    // possibly the pozyx is not connected right. Also make sure the jumper of the boot pins is present.
    status = POZYX_FAILURE;
  }
  
  // readout the selftest registers to validate the proper functioning of pozyx
  if(regRead(POZYX_ST_RESULT, &selftest, 1) == POZYX_FAILURE){
    return POZYX_FAILURE;
  } 

  if(print_result){
//    Serial.print("selftest: 0b");
//    Serial.println(selftest, BIN);
  }

  if((_hw_version & POZYX_TYPE) == POZYX_TAG)
  {
    // check if the uwb, pressure sensor, accelerometer, magnetometer and gyroscope are working
    if(selftest != 0b00111111) {    
      status = POZYX_FAILURE;
    }
  }else if((_hw_version & POZYX_TYPE) == POZYX_ANCHOR)
  {
    // check if the uwb transceiver and pressure sensor are working
    if(selftest != 0b0011000) {    
      status = POZYX_FAILURE;
    }
    return status;
  }
  
  // set everything ready for interrupts
  _interrupt = 0;
  if(_mode == MODE_INTERRUPT){
    // set the function that must be called upon an interrupt
    //attachInterrupt(interrupt_pin, IRQ, RISING);

    // use interrupt as provided and initiate the interrupt mask
    uint8_t int_mask = interrupts;
    if (interrupt_pin == 1){
      int_mask |= POZYX_INT_MASK_PIN;
    }          
    if (regWrite(POZYX_INT_MASK, &int_mask, 1) == POZYX_FAILURE){
      return POZYX_FAILURE;
    }
  }  
  
  // all done
  MyDelayMs(POZYX_DELAY_LOCAL_WRITE);
  return status;
}

/**
  * Reads a number of bytes from the specified pozyx register address using I2C
  */
int regRead(uint8_t reg_address, uint8_t *pData, int size)
{  
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c reads in smaller chunks
  
  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = POZYX_SUCCESS;
  uint8_t reg;
    
  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    reg = reg_address+offset;    
    
    if(i+1 != n_runs){      
      status &= i2cWriteRead(&reg, 1, pData+offset, BUFFER_LENGTH);
    }else{      
      status &= i2cWriteRead(&reg, 1, pData+offset, size-offset);
    }    
  }
  
  return status;
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int regWrite(uint8_t reg_address, const uint8_t *pData, int size)
{  
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c writes in smaller chunks
  
  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;
    
  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    if(i+1 != n_runs){
      status &= i2cWriteWrite(reg_address+offset, pData+offset, BUFFER_LENGTH);
    }else{
      status &= i2cWriteWrite(reg_address+offset, pData+offset, size-offset);
    }    
  }
  
  return status;
}

/**
  * Call a register function using i2c with given parameters, the data from the function is stored in pData
  */
int regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  uint8_t status;
  
  // this feels a bit clumsy with all these memcpy's
  uint8_t write_data[param_size+1];
  write_data[0] = reg_address;
  memcpy(write_data+1, params, param_size);
  uint8_t read_data[size+1];
  
  // first write some data with i2c and then read some data
  status = i2cWriteRead(write_data, param_size + 1, read_data, size+1);
  if(status == POZYX_FAILURE)
    return status;    
  
  memcpy(pData, read_data+1, size);

  
  // the first byte that a function returns is always it's success indicator, so we simply pass this through
  return read_data[0];
}


/**
 * Wirelessly write a number of bytes to a specified register address on a remote Pozyx device using UWB.
 */
int remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_WRITABLE(reg_address))      return POZYX_FAILURE;    // the register is not writable
  if(size > MAX_BUF_SIZE-1)              return POZYX_FAILURE;    // trying to write too much data
  
  int status = 0;
  
  // first prepare the packet to send
  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;              // the first byte is the register address we want to start writing to.
  memcpy(tmp_data+2, pData, size);         // the remaining bytes are the data bytes to be written starting at the register address.
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, size+2, NULL, 0);
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
 
  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x04;    // flag to indicate a register write  
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  
  return status;
}

/**
 * Wirelessly read a number of bytes from a specified register address on a remote Pozyx device using UWB. 
 */
int remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_READABLE(reg_address))      return POZYX_FAILURE;        // the register is not readable
  if(size > MAX_BUF_SIZE)                return POZYX_FAILURE;        // trying to read too much data
  if(destination == 0)                   return POZYX_FAILURE;        // remote read not allowed in broadcast mode
  
  int status = 0;
  
  // first prepare the packet to send
  uint8_t tmp_data[3];
  tmp_data[0] = 0;                  // the offset in the TX buffer
  tmp_data[1] = reg_address;        // the first byte is the register address we want to start reading from
  tmp_data[2] = size;               // the number of bytes to read starting from the register address 
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, 3, NULL, 0);
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
 
  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x02;    // flag to indicate a register read  
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  
  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;
    
  // wait up to x ms to receive a response  
  if(waitForFlag(POZYX_INT_STATUS_RX_DATA, POZYX_DELAY_INTERRUPT))
  {   
    // we received a response, now get some information about the response
    uint8_t rx_info[3]= {0,0,0};
    regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
    uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
    uint8_t data_len = rx_info[2];
    
    if( remote_network_id == destination && data_len == size)
    {
      status = readRXBufferData(pData, size);        
      return status;
    }else{
      return POZYX_FAILURE;  
    }     
    
  }else{
    // timeout
    return POZYX_TIMEOUT;  
  }
}

/*
 * Wirelessly call a register function with given parameters on a remote Pozyx device using UWB, the data from the function is stored in pData
 */
int remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  // some checks
  if(!IS_FUNCTIONCALL(reg_address))      return POZYX_FAILURE;        // the register is not a function  
  
  int status = 0;
  
  // first prepare the packet to send
  uint8_t tmp_data[param_size+2];
  tmp_data[0] = 0;  
  tmp_data[1] = reg_address;                // the first byte is the function register address we want to call.
  memcpy(tmp_data+2, params, param_size);   // the remaining bytes are the parameter bytes for the function.
  status = regFunction(POZYX_TX_DATA, tmp_data, param_size+2, NULL, 0);
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
  
  // send the packet
  uint8_t tx_params[3];
  tx_params[0] = (uint8_t)destination;
  tx_params[1] = (uint8_t)(destination>>8);
  tx_params[2] = 0x08;    // flag to indicate a register function call  
  status = regFunction(POZYX_TX_SEND, tx_params, 3, NULL, 0);
  
  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;
    
  // wait up to x ms to receive a response  
  if(waitForFlag(POZYX_INT_STATUS_RX_DATA, POZYX_DELAY_INTERRUPT))
  {    
    // we received a response, now get some information about the response
    uint8_t rx_info[3];
    regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
    uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
    uint8_t data_len = rx_info[2];
            
    if( remote_network_id == destination && data_len == size+1)
    {
      uint8_t return_data[size+1];
  
      status = readRXBufferData(return_data, size+1);   
      
      if(status == POZYX_FAILURE){
        // debug information
        // Serial.println("could not read from rx buffer");
        return status;    
      }
  
      memcpy(pData, return_data+1, size);
        
      return return_data[0];
    }else{
      // wrong response received.
      // debug information
      /*
      Serial.println("wrong response received. remoteRegFunction");
      Serial.print("Remote id: ");
      Serial.println(remote_network_id, HEX);
      Serial.print("data length: ");
      Serial.println(data_len);
      */

      return POZYX_FAILURE;  
    }     
    
  }else{
    // timeout
    // debug information
    // Serial.println("timeout from ack");
    return POZYX_TIMEOUT;  
  }
}

int writeTXBufferData(uint8_t data[], int size, int offset)
{
  if (offset + size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }
    
  int i, status = 1;  
  int max_bytes = BUFFER_LENGTH-2;
  int n_runs = ceil((float)size / max_bytes);
  uint8_t params[BUFFER_LENGTH];

  // read out the received data.    
  for(i=0; i<n_runs; i++)
  {   
    params[0] = offset + i*max_bytes;      // the offset
    if(i+1 != n_runs){       
      memcpy(params+1, data+i*max_bytes, max_bytes);
      status &= regFunction(POZYX_TX_DATA, params, max_bytes + 1, NULL, 0);    
    }else{
      memcpy(params+1, data+i*max_bytes, size-i*max_bytes);        
      status &= regFunction(POZYX_TX_DATA, params, size-i*max_bytes+1, NULL, 0);          
    }       
  }  

  return status;
}

int readRXBufferData(uint8_t* pData, int size)
{
  if (size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }
  
  int status;
  int i;
  uint8_t params[2];
  int max_bytes = BUFFER_LENGTH-1;
  int n_runs = ceil((float)size / max_bytes);

  // read out the received data.    
  for(i=0; i<n_runs; i++)
  {        
    params[0] = i*max_bytes;      // the offset
    if(i+1 != n_runs){
      params[1] = max_bytes;      // the number of bytes to read      
    }else{
      params[1] = size - i*max_bytes;      // the number of bytes to read      
    }  
    status = regFunction(POZYX_RX_DATA, params, 2, pData+params[0], params[1]);    
  }
  
  return status;
} 

int sendTXBufferData(uint16_t destination)
{
  int status;

  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  MyDelayMs(POZYX_DELAY_LOCAL_FUNCTION);

  return status;
}


/*
 * This function sends some data bytes to the destination
 */
int sendData(uint16_t destination, uint8_t *pData, int size)
{
  if(size > MAX_BUF_SIZE)          return POZYX_FAILURE;        // trying to send too much data
  
  uint8_t status = 0;

  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;                        // the first byte is the offset byte.
  memcpy(tmp_data+1, pData, size);
  
  // set the TX buffer
  status = regFunction(POZYX_TX_DATA, tmp_data, size+1, NULL, 0);  
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
 
  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    // flag to indicate we're just sending data  
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  
  return status;   
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int i2cWriteWrite(const uint8_t reg_address, const uint8_t *pData, int size)
{
	UINT8 i, flag, j;
    flag = 0;
    for (i = 0; i < 100; i++) {
        //1. i2c start
        I2CStart(I2C3);
        //2. Set  in W Mode
        I2CSendByte(I2C3, (POZYX_I2C_ADDRESS << 1) | 0);
        //3. Check ACK
        I2CIdle();
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
    }

    if (!flag) return (FALSE); // Exit if there was a problem
    // 4.if write cmd was successful, put the adress on the bus
    I2CSendByte(I2C3, reg_address);
    I2CIdle();
    for (j = 0; j < size; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            I2CSendByte(I2C3, *(pData + j));
        } else {
            return FALSE;
        }
    }
    I2CStop(I2C3);
	
  return (POZYX_SUCCESS);         // return : no error
}

/**
  * Call a register function using I2C with given parameters
  */
int i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len)
{
    uint8_t i, flag, ret, j;
    flag = 0;
    ret = POZYX_FAILURE;
    for (i = 0; i < 100; i++) // wait for ACK for some time
    {
        //1. i2c start
        I2CStart(I2C3);
        //2. Set Slave in W Mode
        I2CSendByte(I2C3, (POZYX_I2C_ADDRESS << 1) | 0);
        //3. Check ACK
        I2CIdle();
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
    }

    if (!flag) return POZYX_FAILURE; // Exit if there was a problem
    flag = 0;
    // 4.if write cmd was successful, put the regno on the bus
    I2CSendByte(I2C3, write_data[0]);
    I2CIdle();
    for (j = 1; j < write_len; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            I2CSendByte(I2C3, *(write_data + j));
        } else {
            return POZYX_FAILURE;
        }
    }
	
    // Now that the register addres is setup, we can ask the slave to enter read mode.
    for (j = 0; j < 100; j++) {
        //5.Issue a repeated start = a start cond without a prior stop
        I2CStart(I2C3);
        //6. Set Slave in R mode
        I2CSendByte(I2C3, (POZYX_I2C_ADDRESS << 1) | 1);
        //7. Check ACK
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
    }

    if (!flag) return POZYX_FAILURE; // Exit if there was a problem

    for (i = 0; i < read_len; i++) //read all the bytes
    {
        I2CIdle();
        // got the ack, processor is in read mode
        //8. read the register
        I2CCONbits.RCEN = 1; // enable master read
        while (I2CCONbits.RCEN); // wait for byte to be received !(I2CSTATbits.RBF)

        I2CIdle();
        I2CSTATbits.I2COV = 0;
        *(read_data + i) = I2CRCV;

        if ((i + 1) == read_len) {
            //9. Generate a NACK on last byte
            I2CCONbits.ACKDT = 1; // send nack
            I2CCONbits.ACKEN = 1;
            //10. generate a stop
            I2CStop(I2C3);
        } else {
            I2CCONbits.ACKDT = 0; // send ACK for sequential reads
            I2CCONbits.ACKEN = 1;
        }

        ret = POZYX_SUCCESS;
    }

  return (POZYX_SUCCESS);  // return : no error
}


void drvI2CInit(void) {
    UINT16 temp = 0;
    I2CCON = 0; // reset bits to 0
    I2CCONbits.I2CEN = 0; // disable module
    I2CBRG = (GetPeripheralClock() / FCL) - (GetPeripheralClock() / 10000000) - 1; //Formula from datasheet
    //I2CBRG = ((SSMGetCurrentPeripheralClock() / (2 * Fsck)) - 1);
    I2CSTAT = 0;
    I2CCONbits.I2CSIDL = 1; // dont operate in idle mode
    //I2CCONbits.SCLREL = 1;
    I2CCONbits.I2CEN = 1; // enable module
    temp = I2CRCV; // clear RBF flag
#if(I2C_DEBUG == 1)
    UART2PrintString("Configured i2c1\n");
#endif

}



 void I2CIdle(void) {
    UINT8 t = 255;
    /* Wait until I2C Bus is Inactive */
    while (I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN ||
            I2CCONbits.RSEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT || t--);
}

I2C_RESULT I2CStart(I2C_MODULE id ) {
    // wait for module idle
    I2CIdle();
    // Enable the Start condition
    I2CCONbits.SEN = 1;


    // Check for collisions
    if (I2CSTATbits.BCL) {
#if(I2C_DEBUG == 1)
        UART2PrintString("I2C Start Cond Error! \r\n");
        UART2PrintString("CON\n");
        UART2PutHexWord(I2CCON);

        UART2PrintString("STAT\n");
        UART2PutHexWord(I2CSTAT);
#endif
        return (FALSE);
    } else {
        // wait for module idle
        I2CIdle();
        return (TRUE);
    }
}

 void I2CStop(I2C_MODULE id ) {
    int x = 0;
    // wait for module idle
    I2CIdle();
    //initiate stop bit
    I2CCONbits.PEN = 1;

    //wait for hardware clear of stop bit
    while (I2CCONbits.PEN) {
        if (x++ > 50) break;
    }
    I2CCONbits.RCEN = 0;
    // IFS1bits.MI2C1IF = 0; // Clear Interrupt
    I2CSTATbits.IWCOL = 0;
    I2CSTATbits.BCL = 0;
    // wait for module idle
    I2CIdle();
}

 I2C_RESULT I2CSendByte(I2C_MODULE id, BYTE data) {
    //while(I2CSTATbits.TBF); //Wait till data is transmitted.
    // Send the byte
    I2CTRN = data;

    // Check for collisions
    if ((I2CSTATbits.IWCOL == 1)) {
#if(I2C_DEBUG == 1)
        UART2PrintString("IWOCL \n");
        UART2PrintString("CON\n");
        UART2PutHexWord(I2CCON);

        UART2PrintString("STAT\n");
        UART2PutHexWord(I2CSTAT);
#endif
        return (FALSE);
    } else {
        while (I2CSTATbits.TRSTAT); // wait until write cycle is complete
        if ((I2CSTATbits.BCL == 1)) {
#if(I2C_DEBUG == 1)
            UART2PrintString("I2C Tx Error!\n");
            UART2PrintString("CON\n");
            UART2PutHexWord(I2CCON);

            UART2PrintString("STAT\n");
            UART2PutHexWord(I2CSTAT);
#endif
            return (FALSE);
        } else {
            // wait for module idle
            I2CIdle();
            return (TRUE);
        }
    }
}


/**
 * @brief Read data from an I2C slave
 *
 * This function can be used to read one or more sequential registers from a slave.
 * To read multiple registers, the slave must support multi-byte reads.
 * 
 * @param reg The register to start reading from (UINT8)
 * @param rxPtr A pointer to where the read data should be stored (UINT8*)
 * @param len Number of bytes/registers to read
 * @param slave_adr The 7 bit address of the slave without the R/W bits set
 * @return Boolean indicating if operation completed successfully or not
 */
BOOL drvI2CReadRegisters(UINT8 reg, UINT8* rxPtr, UINT8 len, UINT8 slave_adr) {

    UINT8 i, flag, ret, j;
    flag = 0;
    ret = FALSE;
#if(I2C_DEBUG == 1)
    UART2PrintString("\r\n-----Start read reg:-----\r\n");
    UART2PutHex(reg);
    UART2PrintString("\r\n");
#endif
    for (i = 0; i < 100; i++) // wait for ACK for some time
    {
        //1. i2c start
        I2CStart(I2C3);
        //2. Set Slave in W Mode
        I2CSendByte(I2C3, (slave_adr << 1) | 0);
        //3. Check ACK
        I2CIdle();
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return FALSE; // Exit if there was a problem
    flag = 0;
    // 4.if write cmd was successful, put the regno on the bus
    I2CSendByte(I2C3, reg);
    if (I2CSTATbits.ACKSTAT != 0) // Did we receive an ACK ?
    {
#if(I2C_DEBUG == 1)
        UART2PrintString("Error NACK Rxed\n");
#endif
        return FALSE; // Exit if there was a problem
    }
    // Now that the register addres is setup, we can ask the slave to enter read mode.
    for (j = 0; j < 100; j++) {
        //5.Issue a repeated start = a start cond without a prior stop
        I2CStart(I2C3);
        //6. Set Slave in R mode
        I2CSendByte(I2C3, (slave_adr << 1) | 1);
        //7. Check ACK
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return FALSE; // Exit if there was a problem

    for (i = 0; i < len; i++) //read all the bytes
    {
        I2CIdle();
        // got the ack, processor is in read mode
        //8. read the register
        I2CCONbits.RCEN = 1; // enable master read
        while (I2CCONbits.RCEN); // wait for byte to be received !(I2CSTATbits.RBF)

        I2CIdle();
        I2CSTATbits.I2COV = 0;
        *(rxPtr + i) = I2CRCV;

        if ((i + 1) == len) {
            //9. Generate a NACK on last byte
            I2CCONbits.ACKDT = 1; // send nack
            I2CCONbits.ACKEN = 1;
            //10. generate a stop
            I2CStop(I2C3);
        } else {
            I2CCONbits.ACKDT = 0; // send ACK for sequential reads
            I2CCONbits.ACKEN = 1;
        }

        ret = TRUE;
    }
#if(I2C_DEBUG == 1)
    for (i = 0; i < len; i++) {
        UART2PutChar('$');
        UART2PutHex(*(rxPtr + i));
    }
#endif

    return ret;
}

/**
 * @brief Write data into an I2C slave
 *
 * This function can be used to write one or more sequential registers from a slave.
 * To write multiple registers, the slave must support multi-byte writes.
 *
 * @param reg The register to start writing to (UINT8)
 * @param rxPtr A pointer to where the data should be fetched from (UINT8*)
 * @param len Number of bytes/registers to write
 * @param slave_adr The 7 bit address of the slave without the R/W bits set
 * 
 * @return Boolean indicating if operation completed successfully or not
 */
BOOL drvI2CWriteRegisters(UINT8 adr, UINT8* data, UINT8 len, UINT8 slave_adr) {
    UINT8 i, flag, j;
    flag = 0;
    for (i = 0; i < 100; i++) {
        //1. i2c start
        I2CStart(I2C3);
        //2. Set  in W Mode
        I2CSendByte(I2C3, (slave_adr << 1) | 0);
        //3. Check ACK
        I2CIdle();
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            flag = 1;
            break;
        }
#if(I2C_DEBUG == 1)
        UART2PutChar('.');
#endif
    }

    if (!flag) return (FALSE); // Exit if there was a problem
    // 4.if write cmd was successful, put the adress on the bus
    I2CSendByte(I2C3, adr);
    I2CIdle();
    for (j = 0; j < len; j++) {
        if (I2CSTATbits.ACKSTAT == 0) // Did we receive an ACK ?
        {
            I2CSendByte(I2C3, *(data + j));
        } else {
#if(I2C_DEBUG == 1)
            UART2PrintString("Error NACK Rxed\n");
#endif
            return FALSE;
        }
    }
    I2CStop(I2C3);

    return TRUE;

}

/**
 * @brief A wrapper around drvI2CWriteRegisters() to write only a byte of data
 *
 * @param reg The register to start reading from (UINT8)
 * @param byte The byte to write
 * @param slave_adr The 7 bit address of the slave without the R/W bits set
 * @return Boolean indicating if operation completed successfully or not
 */
BOOL drvI2CWriteByte(UINT8 reg, UINT8 byte, UINT8 slave_adr) {
    return ( drvI2CWriteRegisters(reg, &byte, 1, slave_adr));
}
