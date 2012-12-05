 /**
 ******************************************************************************
 *
 * @file       pios_board.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board hardware for the OpenPilot Version 1.1 hardware.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef PIOS_BOARDCOMMON_H_
#define PIOS_BOARDCOMMON_H_


//------------------------
// Pull these in upfront for clarity 
//------------------------
extern uint32_t pios_i2c_flexi_adapter_id;    // PIOS_I2C
extern uint32_t pios_com_telem_rf_id;        // PIOS_COM
#if defined(PIOS_INCLUDE_GPS)
extern uint32_t pios_com_gps_id;             // PIOS_COM, if GPS
#endif
extern uint32_t pios_com_bridge_id;          // PIOS_COM
extern uint32_t pios_com_vcp_id;             // PIOS_COM
extern uint32_t pios_com_telem_usb_id;       // PIOS_COM


//------------------
// Layout for this file should be (as by information and access):
//        Globals above
//        Device Clock and Watchdog - everything is defined from this
//        Device inheritance to pass to PiOS - only adjusted when new or specific 
//               capabilities are added, basicly an ifdef override - flash size etc
//        Bootloader settings
//        PiOS settings - these will be adjusted most often and is the bulk
//-------------------


//-------------------------
// Device Clock Settings  - Not sure why these are defined as PiOS, they are device ...
//-------------------------
#define PIOS_MASTER_CLOCK			72000000    _INHERIT_MASTER_CLOCK
#define PIOS_PERIPHERAL_CLOCK			(PIOS_MASTER_CLOCK / 2)

//------------------------
// Device WATCHDOG_SETTINGS - Not sure why these are defined as PiOS, they are hardware ... 
//------------------------
#define PIOS_WATCHDOG_TIMEOUT    250      _INHERIT_WATCHDOG_TIMEOUT // CC and PiPX differ
#define PIOS_WDG_REGISTER        BKP_DR4  _INHERIT_WATCHDOG_REGISTER   // Odd place to hard define this, this should come from the vendor device header 

//------------------------
// BOOTLOADER_SETTINGS
//------------------------
#define BOARD_READABLE	TRUE
#define BOARD_WRITABLE	TRUE
#define MAX_DEL_RETRYS	3

//------------------------
// PIOS WATCHDOG_SETTINGS
//------------------------
#define PIOS_WDG_ACTUATOR        0x0001   
#define PIOS_WDG_STABILIZATION   0x0002   
#define PIOS_WDG_ATTITUDE        0x0004   
#define PIOS_WDG_MANUAL          0x0008   
#define PIOS_WDG_AUTOTUNE        0x0010 

//-------------------------
// Interrupt Priorities
//-------------------------
#define PIOS_IRQ_PRIO_LOW			12		// lower than RTOS
#define PIOS_IRQ_PRIO_MID			8		// higher than RTOS
#define PIOS_IRQ_PRIO_HIGH			5		// for SPI, ADC, I2C etc...
#define PIOS_IRQ_PRIO_HIGHEST			4 		// for USART etc...

//------------------------
// TELEMETRY
//------------------------
#define TELEM_QUEUE_SIZE         20  _INHERIT_TELEM_QUEUE_SIZE 

//------------------------
// PIOS_LED
//------------------------
#define PIOS_LED_HEARTBEAT	0  _INHERIT_LED_HEARTBEAT
#define PIOS_LED_USB        PIOS_LED_HEARTBEAT

//------------------------
// PIOS_I2C
// See also pios_board.c
//------------------------
#define PIOS_I2C_MAX_DEVS			1  _INHERIT_I2C_MAX_DEVS
#define PIOS_I2C_MAIN_ADAPTER			(pios_i2c_flexi_adapter_id)


//-------------------------
// PIOS_SPI
//
// See also pios_board.c
//-------------------------
#define PIOS_SPI_MAX_DEVS			2  _INHERIT_SPI_MAX_DEVS // differs for CC PipX

//-------------------------
// PIOS_USART
//-------------------------
#define PIOS_USART_MAX_DEVS			2  _INHERIT_USART_MAX_DEVS // differs for CC PipX

//-------------------------
// PIOS_COM
//
// See also pios_board.c
//-------------------------
#define PIOS_COM_MAX_DEVS			3   _INHERIT_COM_MAX_DEVS // differs for CC PipX
#define PIOS_COM_TELEM_RF               (pios_com_telem_rf_id)
#define PIOS_COM_DEBUG                  PIOS_COM_TELEM_RF

#if defined(PIOS_INCLUDE_GPS)
#define PIOS_COM_GPS                    (pios_com_gps_id)
#endif	/* PIOS_INCLUDE_GPS */
#define PIOS_COM_BRIDGE			(pios_com_bridge_id)
#define PIOS_COM_VCP			(pios_com_vcp_id)
#define PIOS_COM_TELEM_USB              (pios_com_telem_usb_id)

//-------------------------
// ADC
// PIOS_ADC_PinGet(0) = Gyro Z
// PIOS_ADC_PinGet(1) = Gyro Y
// PIOS_ADC_PinGet(2) = Gyro X
//-------------------------
#define PIOS_ADC_OVERSAMPLING_RATE		1  _INHERIT_ADC_OVERSAMPLING_RATE  
#define PIOS_ADC_USE_TEMP_SENSOR		1   _INHERIT_ADC_USE_TEMP_SENSOR // differs for CC PipX
#define PIOS_ADC_TEMP_SENSOR_ADC		ADC1   _INHERIT_ADC_TEMP_SENSOR_ADC
#define PIOS_ADC_TEMP_SENSOR_ADC_CHANNEL	1 _INHERIT_ADC_TEMP_SENSOR_ADC_CHANNEL

#define PIOS_ADC_NUM_PINS			3                                 _INHERIT_ADC_NUM_PINS
// Setup if-def test? Not really needed, unused will be discarded by the compiler
#define PIOS_ADC_PIN1_GPIO_PORT			GPIOA			// PA4 (Gyro X)   _INHERIT_ADC_PIN1_GPIO_PORT 
#define PIOS_ADC_PIN1_GPIO_PIN			GPIO_Pin_4		// ADC12_IN4  _INHERIT_ADC_PIN1_GPIO_PIN
#define PIOS_ADC_PIN1_GPIO_CHANNEL		ADC_Channel_4             _INHERIT_ADC_PIN1_GPIO_CHANNEL
#define PIOS_ADC_PIN1_ADC			ADC2                              _INHERIT_ADC_PIN1_ADC
#define PIOS_ADC_PIN1_ADC_NUMBER		1                           _INHERIT_ADC_PIN1_ADC_NUMBER

#define PIOS_ADC_PIN2_GPIO_PORT			GPIOA			// PA5 (Gyro Y)   _INHERIT_ADC_PIN2_GPIO_PORT 
#define PIOS_ADC_PIN2_GPIO_PIN			GPIO_Pin_5		// ADC123_IN5 _INHERIT_ADC_PIN2_GPIO_PIN
#define PIOS_ADC_PIN2_GPIO_CHANNEL		ADC_Channel_5             _INHERIT_ADC_PIN2_GPIO_CHANNEL
#define PIOS_ADC_PIN2_ADC			ADC1                              _INHERIT_ADC_PIN2_ADC
#define PIOS_ADC_PIN2_ADC_NUMBER		2                           _INHERIT_ADC_PIN2_ADC_NUMBER

#define PIOS_ADC_PIN3_GPIO_PORT			GPIOA			// PA3 (Gyro Z)   _INHERIT_ADC_PIN3_GPIO_PORT 
#define PIOS_ADC_PIN3_GPIO_PIN			GPIO_Pin_3		// ADC12_IN3  _INHERIT_ADC_PIN3_GPIO_PIN
#define PIOS_ADC_PIN3_GPIO_CHANNEL		ADC_Channel_3             _INHERIT_ADC_PIN3_GPIO_CHANNEL
#define PIOS_ADC_PIN3_ADC			ADC2                              _INHERIT_ADC_PIN3_ADC
#define PIOS_ADC_PIN3_ADC_NUMBER		2                           _INHERIT_ADC_PIN3_ADC_NUMBER


#define PIOS_ADC_PORTS				{ PIOS_ADC_PIN1_GPIO_PORT, PIOS_ADC_PIN2_GPIO_PORT, PIOS_ADC_PIN3_GPIO_PORT }
#define PIOS_ADC_PINS				{ PIOS_ADC_PIN1_GPIO_PIN, PIOS_ADC_PIN2_GPIO_PIN, PIOS_ADC_PIN3_GPIO_PIN }
#define PIOS_ADC_CHANNELS			{ PIOS_ADC_PIN1_GPIO_CHANNEL, PIOS_ADC_PIN2_GPIO_CHANNEL, PIOS_ADC_PIN3_GPIO_CHANNEL }
#define PIOS_ADC_MAPPING			{ PIOS_ADC_PIN1_ADC, PIOS_ADC_PIN2_ADC, PIOS_ADC_PIN3_ADC }
#define PIOS_ADC_CHANNEL_MAPPING		{ PIOS_ADC_PIN1_ADC_NUMBER, PIOS_ADC_PIN2_ADC_NUMBER, PIOS_ADC_PIN3_ADC_NUMBER }
#define PIOS_ADC_NUM_CHANNELS			(PIOS_ADC_NUM_PINS + PIOS_ADC_USE_TEMP_SENSOR)
#define PIOS_ADC_NUM_ADC_CHANNELS		2   _INHERIT_ADC_NUM_ADC_CHANNELS // differs for CC PipX
#define PIOS_ADC_USE_ADC2			1 _INHERIT_ADC_USE_ADC2 // differs for CC PipX

// Where is RCC inherited from?? Clocks should be defined in respective board files
#define PIOS_ADC_CLOCK_FUNCTION			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE)
#define PIOS_ADC_ADCCLK				RCC_PCLK2_Div8
/* RCC_PCLK2_Div2: ADC clock = PCLK2/2 */
/* RCC_PCLK2_Div4: ADC clock = PCLK2/4 */
/* RCC_PCLK2_Div6: ADC clock = PCLK2/6 */
/* RCC_PCLK2_Div8: ADC clock = PCLK2/8 */
#define PIOS_ADC_SAMPLE_TIME			ADC_SampleTime_239Cycles5   _INHERIT_ADC_SAMPLE_TIME
/* Sample time: */
/* With an ADCCLK = 14 MHz and a sampling time of 239.5 cycles: */
/* Tconv = 239.5 + 12.5 = 252 cycles = 18�s */
/* (1 / (ADCCLK / CYCLES)) = Sample Time (�S) */
#define PIOS_ADC_IRQ_PRIO			PIOS_IRQ_PRIO_LOW

// Currently analog acquistion hard coded at 480 Hz
// PCKL2 = HCLK / 16
// ADCCLK = PCLK2 / 2
#define PIOS_ADC_RATE		(72.0e6 / 1.0 / 8.0 / 252.0 / (PIOS_ADC_NUM_CHANNELS >> PIOS_ADC_USE_ADC2))
#define PIOS_ADC_MAX_OVERSAMPLING               36    _INHERIT_ADC_MAX_OVERSAMPLING

//------------------------
// PIOS_RCVR
// See also pios_board.c
//------------------------
#define PIOS_RCVR_MAX_DEVS                      3     _INHERIT_RCVR_MAX_DEVS
#define PIOS_RCVR_MAX_CHANNELS			12                _INHERIT_RCVR_MAX_CHANNELS
#define PIOS_GCSRCVR_TIMEOUT_MS			100               _INHERIT_GCSRCVR_TIMEOUT_MS

//-------------------------
// Receiver PPM input
//-------------------------                          
#define PIOS_PPM_MAX_DEVS			1                       _INHERIT_PPM_MAX_DEVS
#define PIOS_PPM_NUM_INPUTS                     12    _INHERIT_PPM_NUM_INPUTS

//-------------------------
// Receiver PWM input
//-------------------------
#define PIOS_PWM_MAX_DEVS			1                       _INHERIT_PWM_MAX_DEVS
#define PIOS_PWM_NUM_INPUTS                     6     _INHERIT_PWM_NUM_INPUTS

//-------------------------
// Receiver DSM input
//-------------------------                          
#define PIOS_DSM_MAX_DEVS			2                       _INHERIT_DSM_MAX_DEVS
#define PIOS_DSM_NUM_INPUTS			12                    _INHERIT_DSM_NUM_INPUTS

//-------------------------
// Receiver S.Bus input
//-------------------------
#define PIOS_SBUS_MAX_DEVS			1                    _INHERIT_SBUS_MAX_DEVS
#define PIOS_SBUS_NUM_INPUTS			(16+2)             _INHERIT_SBUS_NUM_INPUTS

//-------------------------
// Servo outputs
//-------------------------
#define PIOS_SERVO_UPDATE_HZ                    50   _INHERIT_SERVO_UPDATE_HZ
#define PIOS_SERVOS_INITIAL_POSITION            0 /* dont want to start motors, have no pulse till settings loaded */

//--------------------------
// Timer controller settings
//--------------------------
#define PIOS_TIM_MAX_DEVS			3                      _INHERIT_TIM_MAX_DEVS 

//-------------------------
// GPIO
//-------------------------

#define PIOS_GPIO_PORTS				{ _INHERIT_GPIO_1_PORT }
#define PIOS_GPIO_PINS				{ _INHERIT_GPIO_1_PIN }
#define PIOS_GPIO_CLKS				{ _INHERIT_GPIO_1_GPIO_CLK }
#define PIOS_GPIO_NUM				1                         _INHERIT_GPIO_NUM	


//-------------------------
// USB
//-------------------------
#define PIOS_USB_HID_MAX_DEVS                   1            _INHERIT_USB_HID_MAX_DEVS 

#define PIOS_USB_ENABLED                        1            _INHERIT_USB_ENABLED
#define PIOS_USB_DETECT_GPIO_PORT               GPIOC        _INHERIT_USB_DETECT_GPIO_PORT
#define PIOS_USB_MAX_DEVS                       1            _INHERIT_USB_MAX_DEVS
#define PIOS_USB_DETECT_GPIO_PIN                GPIO_Pin_15  _INHERIT_USB_DETECT_GPIO_PIN
#endif /* PIOS_BOARDCOMMON_H_ */
