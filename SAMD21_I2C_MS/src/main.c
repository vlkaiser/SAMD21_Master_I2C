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

#define FAST_MODE_PLUS	0x01		// SPEED bit field

/* Prototypes */
void i2c_clock_init(void);
static void pin_set_peripheral_function(uint32_t pinmux);
void i2c_pin_init(void);
void i2c_master_init(void);


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
 * @note				- Assign I/O lines PA08 and PA09 to the SERCOM peripheral function. *						- Will switch the GPIO functionality of an I/O pin to peripheral
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
 * @fn					- i2c_master_init
 * @brief				- initialize the I2C master function
 * @param[in]			- void
 * @return				- void
 *
 * @note				- Configures control registers, baud registers, sets respective interrupt enable bits.
 *						
 ******************************************************************************************************/
void i2c_master_init()
{	/* Configurations while I2C is DISABLED: */	/* Configure SERCOM_I2CM hardware register CTRLA:	*	- SPEED bit field as 0x01, I2C Master runs at Fast mode + - 1MHz
	*	- SDAHOLD bit field as 0x02, SDA hold time is configured for 300-600ns
	*	- RUNSTDBY bit as 0x01, Generic clock is enabled in all sleep modes (any interrupt can wake up the device)
	*	- MODE bitfield to 0x5, SERCOM2 is configured as I2C Master	*/	SERCOM2->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SPEED(FAST_MODE_PLUS)	|								SERCOM_I2CM_CTRLA_SDAHOLD(0x2)			|								SERCOM_I2CM_CTRLA_RUNSTDBY				|								SERCOM_I2CS_CTRLA_MODE_I2C_MASTER;	/* Enable Smart Mode - Will ACK when DATA.DATA is read*/	SERCOM2->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;	/* Synchronization Busy - Writing CTRLB.CMD or CTRLB.FIFOCLR, STATUS.BUSSTATE, ADDR, or DATA when the SERCOM is
	enabled requires synchronization. When written, the SYNCBUSY.SYSOP bit will be set until
	synchronization is complete.*/	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);	/* BAUDLOW sets SCL low time, BAUD sets SCL high time 	   fSCL = 1MHz, fGCLK = 48MHz (default), trise = 100ns.	   Using datasheet calc, BAUD + BAUDLOW = 33 (tlow =~ 2x thigh) */	SERCOM2->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(11) | SERCOM_I2CM_BAUD_BAUDLOW(22);	/* Wait for Sync */	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);	/* Enabled SERCOM2 Peripheral */	SERCOM2->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;	/* SERCOM Enable synchronization busy (Wait) */
	while((SERCOM2->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE));	/* BusState to Idle (Forced) eg when in unknown state*/	SERCOM2->I2CM.STATUS.bit.BUSSTATE = 0x1;	/* Wait for Sync */	while(SERCOM2->I2CM.SYNCBUSY.bit.SYSOP);	/* Enable Interrupt: Master on bus, Slave on Bus [INTterrupt ENable SET 	   Enable Receive Ready Interrupt Master position, slave position pg 610*/	SERCOM2->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;	/* Enable SERCOM2 interrupt handler */
	system_interrupt_enable(SERCOM2_IRQn);}


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

	/* Insert application code here, after the board has been initialized. */

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
