/*****************************************************************************
 *   i2c.c:  I2C C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *   Modifications from Ascending Technologies GmbH
 *
*****************************************************************************/
#include "LPC214x.h"                        /* LPC21xx definitions */
#include "type.h"
#include "irq.h"
#include "i2c.h"

DWORD I2CMasterState = I2C_IDLE;
DWORD I2CSlaveState = I2C_IDLE;

DWORD I2CCmd;
DWORD I2CMode;

BYTE I2CMasterBuffer[BUFSIZE];
BYTE I2CSlaveBuffer[BUFSIZE];
DWORD I2CCount = 0;
DWORD I2CReadLength;
DWORD I2CWriteLength;

DWORD RdIndex = 0;
DWORD WrIndex = 0;

DWORD I2CSTAT;
/* 
From device to device, the I2C communication protocol may vary, 
in the example below, the protocol uses repeated start to read data from or 
write to the device:
For master read: the sequence is: STA,Addr(W),offset,RE-STA,Addr(w),data...STO 
for master write: the sequence is: STA,Addr(W),length,RE-STA,Addr(r),data...STO
Thus, in state 8, the address is always WRITE. in state 10, the address could 
be READ or WRITE depending on the I2CCmd.
*/   

/*****************************************************************************
** Function name:		I2C0MasterHandler
**
** Descriptions:		I2C0 interrupt handler, deal with master mode
**				only.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void I2C0MasterHandler (void) __irq 
{
    BYTE StatValue;

    /* this handler deals with master read and master write only */
    StatValue = I20STAT;
    
    IENABLE;   
    switch ( StatValue )
    {
	case 0x08:			/* A Start condition is issued. */
	I20DAT = I2CMasterBuffer[0];
	I20CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
	I2CMasterState = I2C_STARTED;
	break;
	
	case 0x10:			/* A repeated started is issued */
	if (  I2CCmd == GET_DEVICE_ID || I2CCmd == GET_TEMPERATURE )
	{
	    I20DAT = I2CMasterBuffer[2];
	}
	I20CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
	I2CMasterState = I2C_RESTARTED;
	break;
	
	case 0x18:			/* Regardless, it's a ACK */
	if ( I2CMasterState == I2C_STARTED )
	{
	    I20DAT = I2CMasterBuffer[1+WrIndex];
	    WrIndex++;
	    I2CMasterState = DATA_ACK;
	}
	I20CONCLR = I2CONCLR_SIC;
	break;
	
	case 0x28:	/* Data byte has been transmitted, regardless ACK or NACK */
	case 0x30:
	if ( WrIndex != I2CWriteLength )
	{   
	    I20DAT = I2CMasterBuffer[1+WrIndex]; /* this should be the last one */
	    WrIndex++;
	    if ( WrIndex != I2CWriteLength )
	    {   
		I2CMasterState = DATA_ACK;
	    }
	    else
	    {
		I2CMasterState = DATA_NACK;
		if ( I2CReadLength != 0 )
		{
		    I20CONSET = I2CONSET_STA;	/* Set Repeated-start flag */
		    I2CMasterState = I2C_REPEATED_START;
		}
	    }
	}
	else
	{
	    if ( I2CReadLength != 0 )
	    {
		I20CONSET = I2CONSET_STA;	/* Set Repeated-start flag */
		I2CMasterState = I2C_REPEATED_START;
	    }
	    else
	    {
		I2CMasterState = DATA_NACK;
	    }
	}
	I20CONCLR = I2CONCLR_SIC;
	break;
	
	case 0x40:	/* Master Receive, SLA_R has been sent */
	I20CONCLR = I2CONCLR_SIC;
	break;
	
	case 0x50:	/* Data byte has been received, regardless following ACK or NACK */
	case 0x58:
	I2CMasterBuffer[3+RdIndex] = I20DAT;
	RdIndex++;
	if ( RdIndex != I2CReadLength )
	{   
	    I2CMasterState = DATA_ACK;
	}
	else
	{
	    RdIndex = 0;
	    I2CMasterState = DATA_NACK;
	}
	I20CONSET = I2CONSET_AA;	/* assert ACK after data is received */
	I20CONCLR = I2CONCLR_SIC;
	break;
	
	case 0x20:			/* regardless, it's a NACK */
	case 0x48:
	I20CONCLR = I2CONCLR_SIC;
	I2CMasterState = DATA_NACK;
	break;
	
	case 0x38:			/* Arbitration lost, in this example, we don't
					deal with multiple master situation */
	default:
	I20CONCLR = I2CONCLR_SIC;	
	break;
    }
    
    IDISABLE;
    VICVectAddr = 0;		/* Acknowledge Interrupt */

}


void I2C0_send_motordata(void)
{
	WrIndex=0;
	RdIndex=0;
	I2CWriteLength = 5;
	I2CReadLength = 0;
    I2CMasterBuffer[0] = 0x02;
    I2CMasterBuffer[1] = 100;
    I2CMasterBuffer[2] = 100;
    I2CMasterBuffer[3] = 100;
    I2CMasterBuffer[4] = 1;

    //I20CONSET = I2CONSET_STA;	/* Set Start flag */
    //if ( !I2CStart() ) I2CStop();
    I2CCmd = GET_TEMPERATURE;
	I2CEngine();
}

/*****************************************************************************
** Function name:		I2CStart
**
** Descriptions:		Create I2C start condition, a timeout
**				value is set if the I2C never gets started,
**				and timed out. It's a fatal error. 
**
** parameters:			None
** Returned value:		true or false, return false if timed out
** 
*****************************************************************************/
unsigned int I2CStart( void )
{
    /*--- Issue a start condition ---*/
    I20CONSET = I2CONSET_STA;	/* Set Start flag */
    
    /*--- Wait until START transmitted ---*/

    unsigned int timeout = 0;
    unsigned int returnValue = FALSE;
    while( 1 )
    {
	if ( I2CMasterState == I2C_STARTED )
	{
	    returnValue = TRUE;
	    break;	
	}
	if ( timeout >= MAX_TIMEOUT )
	{
	    returnValue = FALSE;
	    break;
	}
	timeout++;
    }
    return( returnValue );
}

/*****************************************************************************
** Function name:		I2CStop
**
** Descriptions:		Set the I2C stop condition, if the routine
**				never exit, it's a fatal bus error.
**
** parameters:			None
** Returned value:		true or never return
** 
*****************************************************************************/
unsigned int I2CStop( void )
{
    I20CONSET = I2CONSET_STO;      /* Set Stop flag */ 
    I20CONCLR = I2CONCLR_SIC;  /* Clear SI flag */ 
            
    /*--- Wait for STOP detected ---*/
    while( I20CONSET & I2CONSET_STO );
    return TRUE;
}

/*****************************************************************************
** Function name:		I2CInit
**
** Descriptions:		Initialize I2C controller
**
** parameters:			I2c mode is either MASTER or SLAVE
** Returned value:		true or false, return false if the I2C
**				interrupt handler was not installed correctly
** 
*****************************************************************************/
void I2CInit( unsigned int I2cMode ) 
{
    IODIR0|= 0x0C;	/* set port 0.2 and port 0.3 to output, high */
    IOSET0 = 0x0C;

//    PINSEL0 |= (0<<7)|(1<<6)|(0<<5)|(1<<4); //Select SCL0 and SDA0

    /*--- Clear flags ---*/
    I20CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;
    I20CONSET &= (~I2CONSET_STO);

    /*--- Reset registers ---*/
    I20SCLL   = I2SCLL_SCLL;
    I20SCLH   = I2SCLH_SCLH;

// by Xun
    /* set I2C rate to 400kHz, given PCLK = 15MHz. datasheet P.220 */
//    I20SCLL = 19;
 //   I20SCLH = 18;

//    unsigned long flag_irq = install_irq(I2C0_INT, (void *) I2C0MasterHandler );
    I20CONSET = I2CONSET_I2EN;
}

/*****************************************************************************
** Function name:		I2CEngine
**
** Descriptions:		The routine to complete a I2C transaction
**				from start to stop. All the intermitten
**				steps are handled in the interrupt handler.
**				Before this routine is called, the read
**				length, write length, I2C master buffer,
**				and I2C command fields need to be filled.
**				see i2cmst.c for more details. 
**
** parameters:			None
** Returned value:		true or false, return false only if the
**				start condition can never be generated and
**				timed out. 
** 
*****************************************************************************/
unsigned int I2CEngine( void ) 
{
    I2CMasterState = I2C_IDLE;
    RdIndex = 0;
    WrIndex = 0;
    if ( I2CStart() != TRUE )
    {
		I2CStop();
	return ( FALSE );
    }
    while ( 1 )
    {
	if ( I2CMasterState == DATA_NACK )
	{
	    I2CStop();
	    break;
	}
    }    
    return ( TRUE );      
}

/******************************************************************************
**                            End Of File
******************************************************************************/

/* laser with I2C. by Xun */
unsigned int I2C0WaitForSI(void) //Wait till I2C0 block sets SI
{
	int timeout = 0;
	while ( !(I20CONSET & I2CONSET_SI) ) //Wait till SI bit is set. This is important!
	{
		timeout++;
		if (timeout > 10000) return 0; //In case we have some error on bus
	}
	return 1; //SI has been set
}
void I2C0SendStart(void)
{
	unsigned int SI_flag = 0;
	I20CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC; //Clear everything
	I20CONSET = I2CONSET_STA;	/* Set Start flag */
	SI_flag = I2C0WaitForSI(); //Wait till the SI bit is set
}
void I2C0SendStop(void)
{
	int timeout = 0;
	I20CONSET = I2CONSET_STO; //Set stop bit to send a stop condition
	I20CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
	while (I20CONSET & I2CONSET_STO) //Wait till STOP is send. This is important!
	{
		timeout++;
		if (timeout > 10000) //In case we have some error on bus
		{
			printf("STOP timeout!\n");
			return;
		}
	}
	I20CONCLR = I2CONCLR_SIC;
}
void I2C0TX_Byte(unsigned char data)
{
	I20DAT = data;
	I20CONCLR = I2CONCLR_SIC | I2CONCLR_STAC; //Clear These to TX data
	I20CONSET &= (~I2CONSET_STO);
	I2C0WaitForSI(); //wait till TX is finished
}
unsigned char I2C0RX_Byte(void)
{
	I20CONCLR = I2CONCLR_AAC; //Send NACK to stop; I2C block will send a STOP automatically, so no need to send STOP thereafter.
	I20CONCLR = I2CONCLR_SIC; //Clear SI to Start RX
	I2C0WaitForSI(); //wait till RX is finished
	return I20DAT;
}
#define checkStatus(statusCode) \
if(I20STAT!=statusCode) \
{ \
	printf("Failed for status code: %i(decimal), got status code: %i(decimal)\n",statusCode,I20STAT); \
	I2C0SendStop(); return 0; \
}
unsigned int I2C0WriteByte(unsigned int registerAddress, unsigned char data)
{
		I2C0SendStart(); //Send START on the Bus to Enter Master Mode
		checkStatus(0x08); //START sent

		I2C0TX_Byte((0x62<<1) & 0xFE); //Send SlaveAddress + 0 to indicate a write.
		checkStatus(0x18);//SLA+W sent and ack recevied

		I2C0TX_Byte(registerAddress); //send the registerAddress
		checkStatus(0x28); //byte has been sent and ACK recevied

		I2C0TX_Byte(data); //Finally send the data byte.
		checkStatus(0x28); //Data Byte has been sent and ACK recevied

		I2C0SendStop(); //Send STOP since we are done.
		return 1;
}
unsigned int I2C0ReadByte(unsigned int registerAddress, unsigned char *data)
{
	unsigned int RXData = 0;
		I2C0SendStart(); //Send START on the Bus to Enter Master Mode
		I2CSTAT = I20STAT;
		checkStatus(0x08); //START sent

		I2C0TX_Byte((0x62<<1) & 0xFE); //Send SlaveAddress + 0 to indicate a write.
		I2CSTAT = I20STAT;
		checkStatus(0x18);//SLA+W sent and ack recevied

		I2C0TX_Byte(registerAddress);
		I2CSTAT = I20STAT;
		checkStatus(0x28);

		I2C0SendStop();

		I2C0SendStart(); //Send START on the Bus to Enter Master Mode
		I2CSTAT = I20STAT;
		checkStatus(0x08); //START sent

		I2C0TX_Byte((0x62<<1) | 0x01); //This makes SLA-RW bit to 1 which indicates read.
		I2CSTAT = I20STAT;
		checkStatus(0x40); //SLA-R has been Transmitted and ACK received.

		RXData = I2C0RX_Byte();

		*data = (unsigned char)RXData; //Write recieved data to buffer
		printf("Data='%c' ",RXData);
		I2C0SendStop();
	return 1;
}


/* with irq */
void write_byte(unsigned char slave_address, unsigned char register_address, unsigned char data){
	/* initialize Master Transmitter mode */
//	I20CONSET = I2CONSET_I2EN;	// set I2EN bit, clear STA, STO and AA bit

	WrIndex=0;
	I2CWriteLength = 3;
	I2CMasterBuffer[0] = (slave_address << 1) & 0xFE;	// laser address + write bit(0)
    I2CMasterBuffer[1] = register_address;
    I2CMasterBuffer[2] = data;

	// enter Master Transmitter mode
    unsigned int flag_start = I2CStart();

	// Set Stop flag
	while(1){
		if( I2CMasterState == DATA_NACK ){
			unsigned int flag_stop = I2CStop();
			break;
		}
	}
}

unsigned char read_byte(unsigned char slave_address, unsigned char register_address){
	/* initialize Master Receiver mode */
//	I20CONSET = I2CONSET_I2EN;	// set I2EN bit, clear STA, STO and AA bit

	/* write register address */
	WrIndex=0;
	I2CWriteLength = 2;
	I2CMasterBuffer[0] = (slave_address << 1) & 0xFE;	// laser address + write bit(0)
	I2CMasterBuffer[1] = register_address;	// register address

	// enter Master Receiver mode
    unsigned int flag_start = I2CStart();

	// Set Stop flag
	while(1){
		if( I2CMasterState == DATA_NACK ){
			unsigned int flag_stop = I2CStop();
			break;
		}
	}

	/* read one byte */
	RdIndex=0;
	I2CReadLength = 1;
    I2CMasterBuffer[0] = (slave_address << 1) | 0x01;	// laser address + read bit(1)

    flag_start = I2CStart();

	unsigned char byte_read = I2CMasterBuffer[3];

	// Set Stop flag
	while(1){
		if( I2CMasterState == DATA_NACK ){
			unsigned int flag_stop = I2CStop();
			break;
		}
	}

	return byte_read;
}
