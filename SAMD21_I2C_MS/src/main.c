/***************************************************************************************************************************
* Project							: SAMD21 I2C Master 
* Program Name						:
* Author							: vkaiser
* Date Created						: Jan 16 2020
*
* Purpose							: Implement I2C between SAMD20/SAMD21 MCUs; One as SL, one as MS per AT11628
*
*
* MCU								: ATSAMD21J18A
* Language							: C
* Hardware Modifications			: N/A
* Debugger							: EDBG (On-board)
*
* Repo / Revision History			: https://github.com/vlkaiser/
*
* - Special Setup -
*  Header files for all drivers that have been imported from Atmel Software Framework (ASF).
*  Use in conjunction with			: SAMD21 Xplained Pro
*  Wiring Details					: R305, R306 replaced with 1.82Kohm
*
* Revision History					:
* 	Date				Author			Notes
* 						vkaiser			- Initial commit
*
***************************************************************************************************************************/
#include <asf.h>

#define STANDARD_MODE_FAST_MODE	0x0				// I2C Speed Mode Standard
#define FAST_MODE_PLUS			0x01			// I2C SPEED bit field
#define HIGHSPEED_MODE			0x02			// I2C SPEED bit field
#define SLAVE_ADDR				0x12			// SLAVE device Address

//Settings for BAUD Rate and I2C Transaction Speed
#define F_GCLK			48000000			//48MHz
#define F_SCL_STD		100000				//100khz	- w/ STD or FastMode+ = 20khz
#define F_SCL_FAST		400000				//400khz - w/ STD or FastMode+ = 76khz
#define F_SCL_FASTPLUS	1000000				//1Mhz - w/ STANDARD_MODE_FAST_MODE = 243khz
#define F_SCL_HS		3400000				//3.4MHz

#define BUF_SIZE		3					// TX/RX Buffer Size


/* GLOBALS */
uint8_t i;
volatile bool tx_done = false, rx_done = false;
uint8_t tx_buf[BUF_SIZE] = {1, 2, 3};
uint8_t rx_buf[BUF_SIZE];

/* Prototypes */
void i2c_clock_init(void);
static void pin_set_peripheral_function(uint32_t pinmux);
void i2c_pin_init(void);
uint32_t calculate_baud(uint32_t fgclk, uint32_t fscl);
uint32_t calculate_baud_high(uint32_t fgclk, uint32_t fscl);
void i2c_master_init(void);
void i2c_master_transaction(void);

/******************************************************************************************************
 * @fn					- i2c_clock_init
 * @brief				- Configure peripheral bus clock (APB) and generic clock for i2c SERCOM module
 * @param[in]			- void
 * @return				- void
 *
 * @note				- 
 ******************************************************************************************************/
void i2c_clock_init()
{
	struct system_gclk_chan_config gclk_chan_conf;		//struct to configure generic clock for SERCOM
	uint32_t gclk_index = SERCOM2_GCLK_ID_CORE;

	system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBC, PM_APBCMASK_SERCOM2);	//Turn on module in Power Manager - peripheral bus C
	system_gclk_chan_get_config_defaults((&gclk_chan_conf));				//Turn on generic clock for i2c: Default is generator0
	system_gclk_chan_set_config(gclk_index, &gclk_chan_conf);				//Write defaults to SERCOM2
	system_gclk_chan_enable(gclk_index);									//Enable
}

/******************************************************************************************************
 * @fn					- pin_set_peripheral_function
 * @brief				- Initialize i2c pins to SERCOM-Alternate peripheral function (D)
 * @param[in]			- pinmux (MCU driver files for pin definitions)
 * @return				- void
 *
 * @note				- Assign I/O lines PA08 and PA09 to the SERCOM peripheral function. *						- Will switch the GPIO functionality of an I/O pin to peripheral
 *							 functionality and assigns the given peripheral function to the pin.
 ******************************************************************************************************/
static void pin_set_peripheral_function(uint32_t pinmux)
{
	uint8_t port = (uint8_t)((pinmux >> 16)/32);
	PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >> 16) & 0x01u)));
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux &0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));

}

/******************************************************************************************************
 * @fn					- i2c_pin_init
 * @brief				- Initialize i2c pins to SERCOM-Alternate peripheral function (D)
 * @param[in]			- void
 * @return				- void
 *
 * @note				- PA08 = SDA, PA09 = SCL
 ******************************************************************************************************/
void i2c_pin_init()
{
	pin_set_peripheral_function(PINMUX_PA08D_SERCOM2_PAD0);	
	pin_set_peripheral_function(PINMUX_PA09D_SERCOM2_PAD1);
}

/******************************************************************************************************
 * @fn					- calculate_baud
 * @brief				- Calculate the BAUD value using f_{gclk},f_{scl}, t_{rise}
 *						  BAUDLOW = 0
 * @param[in]			- fgclk
 *						- fscl
 *
 * @return				- f_baud
 *
 * @note				- FSCL = fGCLK / (10 + BAUD +BAUDLOW + fGCLKTRISE )
 *						  eg param: fgclk (@48MHz), fscl (@100khz, @1MHz...)
 ******************************************************************************************************/
uint32_t calculate_baud(uint32_t fgclk, uint32_t fscl)
{
	float f_baud;
	f_baud = (((float)fgclk/(float)fscl) - 10 - ((float)fgclk*0.0000003))/2;

	return ((uint32_t)f_baud);
}

/******************************************************************************************************
 * @fn					- calculate_baud_low
 * @brief				- Calculate the BAUDLOW value using f_{gclk},f_{scl}, t_{rise}
 *						  Given BAUDLOW (TLOW) is approx 2x BAUD (THIGH)
 * @param[in]			- fgclk
 *						- fscl
 *
 * @return				- f_baud
 *
 * @note				- FSCL = fGCLK / (10 + BAUD +BAUDLOW + fGCLKTRISE )
 *						  eg param: fgclk (@48MHz -> 4800), fscl (@100khz -> 100, @1MHz -> 1000)
 ******************************************************************************************************/
uint32_t calculate_baud_high(uint32_t fgclk, uint32_t fscl)
{
	float f_temp, f_baud;
	f_temp = ((float)fgclk/(float)fscl) - (((float)fgclk/(float)1000000)*0.3);
	f_baud = (f_temp/2)-5;

	return ((uint32_t)f_baud);
}


/******************************************************************************************************
 * @fn					- i2c_master_init
 * @brief				- initialize the I2C master function
 * @param[in]			- void
 * @return				- void
 *
 * @note				- Configures control registers, baud registers, sets respective interrupt enable bits.
 *						
 ******************************************************************************************************/
void i2c_master_init()
{
	/* Configurations while I2C is DISABLED: */

	/* Configure SERCOM_I2CM hardware register CTRLA:
	*	- SPEED bit field as 0x01, I2C Master runs at Fast mode + - 1MHz
	*	- SDAHOLD bit field as 0x02, SDA hold time is configured for 300-600ns
	*	- RUNSTDBY bit as 0x01, Generic clock is enabled in all sleep modes (any interrupt can wake up the device)
	*	- MODE bitfield to 0x5, SERCOM2 is configured as I2C Master
	*/
	SERCOM2->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SPEED(STANDARD_MODE_FAST_MODE)	|
								SERCOM_I2CM_CTRLA_SDAHOLD(0x2)			|
								SERCOM_I2CM_CTRLA_RUNSTDBY				|
								//SERCOM_I2CM_CTRLA_SCLSM					|
								SERCOM_I2CS_CTRLA_MODE_I2C_MASTER;

	/* Enable Smart Mode - Will ACK when DATA.DATA is read*/
	SERCOM2->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;

	/* Synchronization Busy - Writing CTRLB.CMD or CTRLB.FIFOCLR, STATUS.BUSSTATE, ADDR, or DATA when the SERCOM is
	enabled requires synchronization. When written, the SYNCBUSY.SYSOP bit will be set until synchronization is complete.*/
	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

	/* BAUDLOW sets SCL low time, BAUD sets SCL high time 
	   fSCL = 1MHz, fGCLK = 48MHz (default), trise = 100ns.
	   Using datasheet calc, BAUD + BAUDLOW = 33 (tlow =~ 2x thigh) 
	   BAUDLOW = 0, BAUD.BAUD sets SCL High and SCL Low, trise (std) = 3us?
	   */
	//SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(11) | SERCOM_I2CM_BAUD_BAUDLOW(22);
	SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(calculate_baud(F_GCLK, F_SCL_FAST)) | SERCOM_I2CM_BAUD_BAUDLOW(0);

	/* Wait for Sync */
	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

	/* Enabled SERCOM2 Peripheral */
	SERCOM2->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;

	/* SERCOM Enable synchronization busy (Wait) */
	while((SERCOM2->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE));

	/* BusState to Idle (Forced) eg when in unknown state*/
	SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x1;

	/* Wait for Sync */
	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

	/* Enable Interrupt: Master on bus, Slave on Bus [INTterrupt ENable SET 
	   Enable Receive Ready Interrupt Master position, slave position pg 610*/
	SERCOM2->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;

	/* Enable SERCOM2 interrupt handler */
	system_interrupt_enable(SERCOM2_IRQn);

}

/******************************************************************************************************
 * @fn					- i2c_master_transaction
 * @brief				- Perform a transaction with the connected slave device
 * @param[in]			- void
 * @return				- void
 *
 * @note				- 
 *						
 ******************************************************************************************************/
void i2c_master_transaction(void)
{
	i = 0;
	
	/* Acknowledge behavior: 0 = send ACK in ACKACT bit CTRLB */
	SERCOM2->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

	/* Wait for Sync */	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

	/* load I2C Slave Address into reg, and Write(0) in 0th bit to Slave.  Initiate Transfer */
	SERCOM2->I2CM.ADDR.reg = (SLAVE_ADDR << 1) | 0;
	while(!tx_done);			//wait for transmit complete (Interrupt Handler)
	i =0;

	/* ACK is sent */
	SERCOM2->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

	/* Wait for Sync */	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

	/* Read (1) in 0th bit, from Slave (ACK) */
	SERCOM2->I2CM.ADDR.reg = (SLAVE_ADDR << 1) | 1;
	while(!tx_done);			//wait for transmit complete (Interrupt Handler)

	/* Interrupts are cleared MS/SL */
	SERCOM2->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_MB | SERCOM_I2CM_INTENCLR_SB;
}

/******************************************************************************************************
 * @fn					- SERCOM2_Handler
 * @brief				- After transmitting address to slave and receiving ACK/NACK
 * @param[in]			- void
 * @return				- void
 *
 * @note				- Interrupt handler during while(tx_done == true) in master_transaction
 *						- Overrides weak definition
 ******************************************************************************************************/
 void SERCOM2_Handler(void)
 {
	/* Check for master-on-bus interrupt set condition */
	if (SERCOM2->I2CM.INTFLAG.bit.MB)
	{
		/* Finished TX? (No more i to send?) */
		if (i == BUF_SIZE)
		{
			/* After transferring the last byte, send stop condition */
			SERCOM2->I2CM.CTRLB.bit.CMD = 0x3;
		
			/* Wait for Sync */			while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

			tx_done = true;
			i = 0;
		} else {
			/* Not done. Place the data from the TX buffer to the DATA register */
			SERCOM2->I2CM.DATA.reg = tx_buf[i++];
			while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);	
		}
	}
	/* Check for slave-on-bus interrupt set condition */
	if (SERCOM2->I2CM.INTFLAG.bit.SB)
	{
		/* Finished RX? (No more i to send?) */
		if (i == (BUF_SIZE - 1))
		{
				/* NACK Should be sent BEFORE reading the last byte */
				SERCOM2->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
							/* Wait for Sync */				while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
			
				/* Send stop condition */
				SERCOM2->I2CM.CTRLB.bit.CMD = 0x3;

				/* Wait for Sync */
				while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);	
			
				/* Read last Data byte from Register into buffer */	
				rx_buf[i++] = SERCOM2->I2CM.DATA.reg;

				/* Wait for Sync */
				while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
			
				rx_done = true;
			
			} else {
				/* Not done. Place the data from the DATA register into the RX BUFFER */
				SERCOM2->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
			
				/* Wait for Sync */
				while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
			
				/* Read data from Register into buffer */
				rx_buf[i++] = SERCOM2->I2CM.DATA.reg;
			
				/* Wait for Sync */
				while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);

				/* Send ACK after reading Each Byte */
				SERCOM2->I2CM.CTRLB.bit.CMD = 0x2;

				/* Wait for Sync */
				while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);
		}
	}
}
	

/******************************************************************************************************
 * @fn					- MAIN
 * @brief				- 
 * @param[in]			- void
 * @return				- void
 *
 * @note				- 
 ******************************************************************************************************/
int main (void)
{
	/* Configure clock sources, GLK generators and board hardware */
	system_init();
	i2c_clock_init();
	i2c_pin_init();
	i2c_master_init();
	i2c_master_transaction();

	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, so turn LED on. */
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
		} else {
			/* No, so turn LED off. */
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
		}
	}
}