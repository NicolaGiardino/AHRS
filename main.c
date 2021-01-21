/***********************************************************************************
* This file is part of The AHRS Project.                                           *
*                                                                                  *
* Copyright © 2020 By Nicola di Gruttola Giardino. All rights reserved.            *
* @mail: nicoladgg@protonmail.com                                                  *
*                                                                                  *
* AHRS is free software: you can redistribute it and/or modify                     *
* it under the terms of the GNU General Public License as published by             *
* the Free Software Foundation, either version 3 of the License, or                *
* (at your option) any later version.                                              *
*                                                                                  *
* AHRS is distributed in the hope that it will be useful,                          *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                    *
* GNU General Public License for more details.                                     *
*                                                                                  *
* You should have received a copy of the GNU General Public License                *
* along with The AHRS Project.  If not, see <https://www.gnu.org/licenses/>.       *
*                                                                                  *
* In case of use of this project, I ask you to mention me, to whom it may concern. *
***********************************************************************************/

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "chprintf.h"
#include "usrlib/IMU.h"
#include "usrlib/GPS_Lib.h"

/* Definition of Macros */

#define SPI_BUFFERS_SIZE  51 //DIMENSIONS BUFFER
#define maxFrequency 10000
#define minFrequency 7500
#define maxtemp 40
#define mintemp 30
#define cls(chp)  chprintf(chp, "\033[2J\033[1;1H")
#define GPSREADY 1
#define GPSNOTREADY 0
#define idYPR 0x12
#define idSPD 0x13

/* Define Static Variables */

static BaseSequentialStream* chp = (BaseSequentialStream*)&SD3;
static int temp_error=0;
static uint8_t rxbuf[SPI_BUFFERS_SIZE];

static int Release_Thread=0;

/* Define Global Variables */

int index;
/* Exit messages */
static int ERROR_THREAD = -1;
static int OK = 0;

static int SLEEP_MS = 500;


HG1120CM hg1120;
float YPR[3];
float v[3];
char rxUART;

/* Peripherial Configurations */

/* Configurazione Serial driver con baud rate 115200 */
static const SerialConfig mySerialConfig =
{
  115200,
  0,
  USART_CR2_STOP1_BITS,
  0
};

struct can_instance
{
  CANDriver     *canp;
  uint32_t      led;
};

static const struct can_instance can1 = {&CAND1, 0};

static const CANConfig cancfg = 
{
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_LBKM | CAN_BTR_SJW(1) | CAN_BTR_TS2(1) |
  CAN_BTR_TS1(8) | CAN_BTR_BRP(7)
};

//CFG SPI IMU
static const SPIConfig hs_spicfg = 
{
  false,
  NULL,
  GPIOE,
  GPIOE_ARD_D5,
 0b0000000000011111,
  0
};

//CFG UART GPS
static UARTConfig uart_cfg = 
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	0,
	9600,
	0,
	USART_CR2_LINEN,
	0
};



/*
 *=============================================================================*
 *                                  THREAD
 *=============================================================================*
 */

/*
 * ************************************************************************* *
 *                              THREAD CAN TX
 * ************************************************************************* *
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) 
{
  (void)p;
  CANTxFrame txmsg;
  chRegSetThreadName("Transmitter");

  while (true) 
  {
	  txmsg.IDE = CAN_IDE_EXT;
 	  txmsg.EID = idYPR;
 	  txmsg.RTR = CAN_RTR_DATA;
 	  txmsg.DLC = 6;
 	  txmsg.data16[0] = (int)(YPR[0]*100);
	  txmsg.data16[1] = (int)(YPR[1]*100);
	  txmsg.data16[2] = (int)(YPR[2]*100);
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100));
    chThdSleepMilliseconds(SLEEP_MS);

	  txmsg.IDE = CAN_IDE_EXT;
 	  txmsg.EID = idSPD;
 	  txmsg.RTR = CAN_RTR_DATA;
 	  txmsg.DLC = 6;
 	  txmsg.data16[0] = (int)(v[0]*100);
	  txmsg.data16[1] = (int)(v[1]*100);
	  txmsg.data16[2] = (int)(v[2]*100);
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100));
    chThdSleepMilliseconds(SLEEP_MS);
  }
  if (Release_Thread) chThdExit((msg_t)OK);
  else chThdExit((msg_t)ERROR_THREAD);
}
/*
 * ************************************************************************* *
 *                          THREAD LETTURA IMU VIA SPI
 * ************************************************************************* *
 */
static THD_WORKING_AREA(spi_thread_1_wa, 256);
static THD_FUNCTION(spi_thread_1, p) 
{

  (void)p;
  chRegSetThreadName("SPI thread 1");
  while (true) 
  {
	  index++;
	  /* Slave selection and data exchange.*/
	  if(palReadPad(GPIOF, GPIOF_ARD_D7)==1)
    {
		  spiAcquireBus(&SPID4);
		  spiStart(&SPID4,&hs_spicfg);
		  spiSelect(&SPID4);
		  spiReceive(&SPID4, SPI_BUFFERS_SIZE, rxbuf);
		  spiUnselect(&SPID4);
		  spiStop(&SPID4);
		  spiReleaseBus(&SPID4);
		  Deserialize(rxbuf, 1, &hg1120, 0x04);
		  MadgwickAHRSupdate(hg1120.AngularRate[0],hg1120.AngularRate[1],hg1120.AngularRate[2],hg1120.LinearAcceleration[0],hg1120.LinearAcceleration[1],hg1120.LinearAcceleration[2],hg1120.MagField[0],hg1120.MagField[1],hg1120.MagField[2]);
		  vCalculateYPR(YPR);
		  if(index==100)
      {
			  //1Hz Frequency for GPS
			  uartAcquireBus(&UARTD7);
			  uartStart(&UARTD7,&uart_cfg);
			  do
        {
			  	uartStartReceiveI(&UARTD7, 1, &rxUART);
			  } while(!GPSRead(rxUART));
			  uartStopReceiveI(&UARTD7);
			  uartStop(&UARTD7);
			  uartReleaseBus(&UARTD7);
			  index=0;
			  vCalculate_velocity(v,GPSREADY);
		  } 
      else 
      {
			  vCalculate_velocity(v,GPSNOTREADY);
		  }
		  //1KHz Frequency for IMU
		  chThdSleepMilliseconds(1);
	  }
	  if (Release_Thread) chThdExit((msg_t)OK);
	  else chThdExit((msg_t)ERROR_THREAD);
  }
}

/*
 *============================================================================*
 *                                  MAIN                                      *
 *============================================================================*
 */
int main(void) 
{
  halInit();
  chSysInit();
  vSetup_Kalman();
  index=0;
  /*
   * ************************************************************************* *
   *                            INIT PERIFERICHE                               *
   * ************************************************************************* *
   */
  canInit();
  /*
   * ************************************************************************* *
   *                            START PERIPHERIALS                             *
   * ************************************************************************* *
   */
  //Activates the serial driver 3 using the driver default configuration.
  sdStart(&SD3, &mySerialConfig);
  //Activates the CAN drivers 1.
  canStart(&CAND1, &cancfg);

  /*
   * ************************************************************************* *
   *                      SET PIN PERIPHERIAL                                  *
   * ************************************************************************* *
   */
  //CAN
  palSetPadMode(GPIOD, GPIOD_ZIO_D67, PAL_MODE_ALTERNATE(9));
  palSetPadMode(GPIOD, GPIOD_ZIO_D66, PAL_MODE_ALTERNATE(9));
  //Virtual com port configuratin pins
  palSetPadMode(GPIOD, GPIOD_USART3_TX, PAL_MODE_ALTERNATE(7)
                | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_PUSHPULL);
  palSetPadMode(GPIOD, GPIOD_USART3_RX, PAL_MODE_ALTERNATE(7)
                | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_PUSHPULL);
  //PIN SPI IMU
  palSetPadMode(GPIOE, GPIOE_ZIO_D39, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);   //SPI4_SCK
  palSetPadMode(GPIOE, GPIOE_ARD_D5, PAL_MODE_OUTPUT_PUSHPULL);   //SPI4_NSS
  palSetPadMode(GPIOE, GPIOE_ARD_D3, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);   //SPI4_MISO
  palSetPadMode(GPIOE, GPIOE_ZIO_D38, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);   //SPI4_MOSI
  palSetPadMode(GPIOF, GPIOF_ARD_D7, PAL_MODE_INPUT);//DRDY PIN
  //PIN UART
  palSetPadMode(GPIOC, 10U, PAL_MODE_ALTERNATE(7));//UART5_TX
  palSetPadMode(GPIOC, 11U, PAL_MODE_ALTERNATE(7));//UART5_RX
  TS_ON = 1;

  /*
   * ************************************************************************* *
   *                            CREATE THREAD                                  *
   * ************************************************************************* *
   */
  cls(chp);
  CAN_TX = chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 9,can_rx, (void *)&can1);
  CAN_RX = chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 8,can_tx, NULL);
  SPIIMU = chThdCreateStatic(spi_thread_1_wa, sizeof(spi_thread_1_wa), NORMALPRIO + 4, spi_thread_1, NULL);
  while (true) 
  {
    chThdSleepMilliseconds(500);
  }
}
