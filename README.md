# SAMD21_Master_I2C
Using one SAMD20 and one SAMD21 Xplained pro demo boards, (one as Master, one as Slave) and communicating over I2C.  Example from AT11628

## Documents:
[AT11628 App Note](https://www.avrfreaks.net/sites/default/files/forum_attachments/Atmel-42631-SAM-D21-SERCOM-I2C-Configura.pdf)

[DS40001882D Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf)

## Create New Project
Starting with the Demo Board, ideally port to Userboard
1. New GCC C ASF Board Project
2. Choose SAMD21J18 -> Xplained Pro ATSAMD21J18A and verify LED Toggle with SW0 is working.

## Overview
* Master will transmit a data buffer of a few bytes to the slave.
* Slave will re-transmit the same data buffer to the master.
* Using SERCOM2 (PA08 = SDA, PA09 = SCL).  These I2C lines are available on all 3 EXT headers on the Xplained Pro board, can use any.

## Hardware
* SAMD21 Xplained pro has on-board 4.7k pullup resistors on I2C lines (R305, R306) (Left of PB06 on EXT1) which have been replaced with 2k pullup resistors to improve signal timing. 
<a href="https://www.codecogs.com/eqnedit.php?latex=Rp_{max}&space;=&space;[t_{r}&space;*&space;(0.8473&space;*&space;C_{bus})]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Rp_{max}&space;=&space;[t_{r}&space;*&space;(0.8473&space;*&space;C_{bus})]" title="Rp_{max} = [t_{r} * (0.8473 * C_{bus})]" /></a>

For the SAMD21J18A:  wrost case (Fast Mode) tr = 100ns, Cb = 400pF, 

## Configuration
### Clocks
Master and slave application uses OSC8M as the clock source for Generator 0

### I2C Pin and Register Init (I2C_Master_Init)
Take care - there is an error in 5.1.6 regarding the pin_set_peripheral_function.  See source code
* Switch pin functionality to peripheral function D (SERCOM-ALT: i2c) per datasheet 
Follow I2C Configuration Steps in Datasheet (pg 553)

* Configure CTRL Reg A: i2C Fast Mode Plus, SDA Hold (300-600ns); Runstandby clock enabled in sleep; I2C Config as Master
* Configure CTRL Reg B: SMart Mode Enabled
* Configure Sync busy (wait until not busy)
* Configure BAUD rate: We are configuring the SCL clock.  If we want to control the Low period, we need BAUDLOW:
<a href="https://www.codecogs.com/eqnedit.php?latex=f_{SCL}&space;=&space;\frac{f_{GCLK}}{10&space;&plus;&space;BAUD&space;&plus;&space;BAUDLOW&space;&plus;&space;f_{GCLK}\cdot&space;T_{RISE}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{SCL}&space;=&space;\frac{f_{GCLK}}{10&space;&plus;&space;BAUD&space;&plus;&space;BAUDLOW&space;&plus;&space;f_{GCLK}\cdot&space;T_{RISE}}" title="f_{SCL} = \frac{f_{GCLK}}{10 + BAUD + BAUDLOW + f_{GCLK}\cdot T_{RISE}}" /></a>)
So if fSCL = 1MHz, fGCLK = 48MHz and trise = 100ns we can calculate that BAUD + BAUDLOW = about 33
Convention seems to be that t_{low} is approx 2x t_{high} (per data sheets)
Therefore, BAUD = 11, BAUDLOW = 22.

### Note: It is recommended to set the slave's baud rate higher than the master baud rate and give a sufficient margin to meet the I2C timing.

Interrupt Enable on Master-on-bus and Slave-on-bus. pg610/611 of datasheet
Enable SERCOM interrupts

## Transactions
### Setup
1. Create a global iteration count variable (uint8_t i)
2. Create global boolean values rx_done, tx_done (init to false)
### CTRLB Register - ACK
Setup the ACK behavio.  The ACKACT field is used to define the I2C master’s acknowledge behavior after a data
byte is received from the I2C slave. 
The acknowledge action will be executed when a command is written to CTRLB.CMD bits or after a transfer with Smart Mode is enabled.  After a data byte is received.
Wait for sync

### Interrupt Handler (breakout from master_transaction)
This will override the weak definition of the interrupt handler from the device specific header files

Once the Slave Address has been written to the register, we have initiated a transfer and triggered the interrupt.
The SERCOM2_Handler controls the data transfer process of buffer length (counted by i ) until all data has been transferred.  Then it sets tx_done to true, and returns to the master_transaction function, exiting the while loop.

Repeat similarly for RX: Continue sending bytes until complete. Check for last byte and send ACK/STOP when appropriate.  Return to transaction function.

### Additional
There's a calculate Baud function (we don't use it)
Any globals or #defines used in the functions need to be added, as well as any prototypes.
Otherwise MASTER seems to be completed.  Compiles and loads successfully.









