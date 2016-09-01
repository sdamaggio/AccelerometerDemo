/* _________________________
 *
 *   Innoseis CONFIDENTIAL
 * _________________________
 *
 *  Copyright 2013-2016 Innoseis B.V.
 *
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Innoseis B.V..
 * The intellectual and technical concepts contained herein are
 * proprietary to Innoseis and may be covered by Dutch and Foreign
 * Patents, patents in process, and are protected by trade secret
 * or copyright law.
 *
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Innoseis B.V.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "lis2.h"

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;


/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1


/*Common addresses definition for accelerometer LIS2HH12. */
#define _LIS2_I2CADDR				0x1E	// 7bit I2C address when pin SA0 is low, 0x1D when SA0 is high

#define _LIS2_REG_TEMP_L		0x0B
#define _LIS2_REG_TEMP_H		0x0C
#define _LIS2_REG_WHO_AM_I		0x0F
#define _LIS2_REG_ACT_THS		0x1E
#define _LIS2_REG_ACT_DUR		0x1F
#define _LIS2_REG_CTRL1			0x20
#define _LIS2_REG_CTRL2			0x21
#define _LIS2_REG_CTRL3			0x22
#define _LIS2_REG_CTRL4			0x23
#define _LIS2_REG_CTRL5			0x24
#define _LIS2_REG_CTRL6			0x25
#define _LIS2_REG_CTRL7			0x26
#define _LIS2_REG_STATUS		0x27
#define _LIS2_REG_OUT_X_L		0x28
#define _LIS2_REG_OUT_X_H		0x29
#define _LIS2_REG_OUT_Y_L		0x2A
#define _LIS2_REG_OUT_Y_H		0x2B
#define _LIS2_REG_OUT_Z_L		0x2C
#define _LIS2_REG_OUT_Z_H		0x2D
#define _LIS2_REG_FIFO_CTRL		0x2E
#define _LIS2_REG_FIFO_SRC		0x2F
#define _LIS2_REG_IG_CFG1		0x30
#define _LIS2_REG_IG_SRC1		0x31
#define _LIS2_REG_IG_THS_X1		0x32
#define _LIS2_REG_IG_THS_Y1		0x33
#define _LIS2_REG_IG_THS_Z1		0x34
#define _LIS2_REG_IG_DUR1		0x35
#define _LIS2_REG_IG_CFG2		0x36
#define _LIS2_REG_SRC2			0x37
#define _LIS2_REG_IG_THS2		0x38
#define _LIS2_REG_IG_DUR2		0x39
#define _LIS2_REG_XL_REFERENCE	0x3A
#define _LIS2_REG_XH_REFERENCE	0x3B
#define _LIS2_REG_YL_REFERENCE	0x3C
#define _LIS2_REG_YH_REFERENCE	0x3D
#define _LIS2_REG_ZL_REFERENCE	0x3E
#define _LIS2_REG_ZH_REFERENCE	0x3F

#define	_LIS2_ODR_MASK			0x8F
#define	_LIS2_ODR_SHIFT			4

int16_t lis2Accel[3];
float lis2Tilt;

/* TWI instance. */
static const nrf_drv_twi_t m_twi_LIS2 = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief I2C initialization.
 */
static void twi_init()
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_LIS2_config = { .scl =
			ARDUINO_I2C_SCL_PIN, .sda = ARDUINO_I2C_SDA_PIN, .frequency =
			NRF_TWI_FREQ_100K, .interrupt_priority = APP_IRQ_PRIORITY_HIGH };

	err_code = nrf_drv_twi_init(&m_twi_LIS2, &twi_LIS2_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi_LIS2);
}

/**
 * @brief Function to initialize the lis2 accelerometer
 *
 * Default initialization include +/-2g sensitivity, odr set to LIS2_POWERDOWN,
 * FIFO set to Bypass mode and interrupts disabled
 */
void lis2Init()
{
	twi_init();
}

/**
 * @brief Function to read lis2 accelerometer register
 *
 * @param regAddr register address to be read
 */
static int _lis2ReadReg(uint8_t regAddr)
{
	uint8_t res = 0;
	ret_code_t errCode;

	errCode = nrf_drv_twi_tx(&m_twi_LIS2, _LIS2_I2CADDR, &regAddr, sizeof(regAddr), true);
	APP_ERROR_CHECK(errCode);

	errCode = nrf_drv_twi_rx(&m_twi_LIS2, _LIS2_I2CADDR, (uint8_t*)&res, sizeof(res));
	APP_ERROR_CHECK(errCode);
	//NRF_LOG_PRINTF("\n\rnrf_drv_twi_rx result:%02x err_code: %d\r\n", res, err_code);

	return res;
}

/**
 * @brief Function to write lis2 accelerometer register
 *
 * @param regAddr	address of the register to be written
 * @param regValue 	register value to be written
 */
static void _lis2WriteReg(uint8_t regAddr, uint8_t regValue)
{
	APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi_LIS2, _LIS2_I2CADDR, (uint8_t[]){regAddr, regValue}, sizeof((uint8_t[]){regAddr, regValue}), true));
}

/**
 * @brief Function to set lis2 output data rate
 *
 * Set by default to LIS2_POWERDOWN.
 * The higher the frequency the higher the power consumption.
 * 
 * @param	odr selected Output Data Rate from available odr in enum Lis2OutputDataRate
 * @return	true if successful, false otherwise
 */
bool lis2SetOutputDataRate(Lis2OutputDataRate odr)
{
	static uint8_t regCtrl1 = -1;
	static uint8_t odrValue = 0;

	regCtrl1 = _lis2ReadReg(_LIS2_REG_CTRL1);

	if(regCtrl1 == -1) {
		NRF_LOG_PRINTF("set_odr(): LIS2HH12_REG_CTRL1 read unsuccessful!");
		return false;
	}

	switch(odr) {
		case LIS2_POWERDOWN: odrValue = 0;
			break;
		case LIS2_ODR10HZ: odrValue = 1;
			break;
		case LIS2_ODR50HZ: odrValue = 2;
			break;
		case LIS2_ODR100HZ: odrValue = 3;
			break;
		case LIS2_ODR200HZ: odrValue = 4;
			break;
		case LIS2_ODR400HZ: odrValue = 5;
		  	break;
		case LIS2_ODR800HZ: odrValue = 6;
		  	break;
	    default: NRF_LOG_PRINTF("lis2SetOutputDataRate: Invalid odr value\n"); return false;
	    	break;
	}

	_lis2WriteReg(_LIS2_REG_CTRL1, (regCtrl1 & _LIS2_ODR_MASK) | (odrValue << _LIS2_ODR_SHIFT));

	return true;
}


/**
 * @brief Function to update lis2 temperature reading
 *
 * WARNING: value seems to not make sense!
 * Temperature value is stored in 2 8bit registers, 16 bit total
 * Resolution is 11 bit expressed in two's complement
 * Range is from -40 to +85 degC so 125 total, 0 is at ((40+85)/2)+40=22.5degC
 * Ratio is then 0.061035degC/bit
 * Tout = (int16_t)value * ratio + 22.5degC
 *
 * @return	int16_t temperature register value
 */
int16_t lis2GetTemp()
{
	int16_t temp_val = 0;

	temp_val = (_lis2ReadReg(_LIS2_REG_TEMP_H) << 8 | _lis2ReadReg(_LIS2_REG_TEMP_L));
	//NRF_LOG_PRINTF("temp register: %d\r\n", temp_val);

	return temp_val;
}

/**
 * @brief Function to calculate lis2 tilt angle from acceleration readings
 *
 * @return tilt angle in degrees
 */
static float _lis2GetTilt()
{
	/* tan(tilt) = sqrt(x^2+y^2)/z
	 * tilt = atan( sqrt(x^2+y^2) / z )
	 *
	 * par1 = sqrt(x^2+y^2)
	 */
	float par1 = sqrtf(powf(lis2Accel[0],2) + powf(lis2Accel[1],2));
	float tilt = atan2f(par1, lis2Accel[2]);

	// convert from radians to degrees
	return tilt * (180.0f / (float)M_PI);
}


/**
 * @brief Function to update lis2 x, y and z acceleration readings
 */
void lis2Update()
{
	// 8+8=16 bit registers contain a signed int value which min is -2g (-32768 or -2^15) and max is +2g (32767)
	int16_t regX = (_lis2ReadReg(_LIS2_REG_OUT_X_H) << 8 | _lis2ReadReg(_LIS2_REG_OUT_X_L));
	int16_t regY = (_lis2ReadReg(_LIS2_REG_OUT_Y_H) << 8 | _lis2ReadReg(_LIS2_REG_OUT_Y_L));
	int16_t regZ = (_lis2ReadReg(_LIS2_REG_OUT_Z_H) << 8 | _lis2ReadReg(_LIS2_REG_OUT_Z_L));

	/* input range is (-2^15, +2^15-1)
	 * output range is (-2000mg, +1999mg)
	 * ratio is 16,384 bit/g or (2^14/1000) bit/g
	 *
	 * multiply for 1/ratio instead of dividing
	 */
	int32_t xVal = (regX * 1000) >> 14;
	int32_t yVal = (regY * 1000) >> 14;
	int32_t zVal = (regZ * 1000) >> 14;

	lis2Accel[0] = (int16_t)xVal;
	lis2Accel[1] = (int16_t)yVal;
	lis2Accel[2] = (int16_t)zVal;

	// update tilt angle
	lis2Tilt = _lis2GetTilt();
}

/**
 * @brief Function for main application entry.
 */
int main(void) {

	lis2Init();
	nrf_delay_ms(50);
	lis2SetOutputDataRate(LIS2_ODR10HZ);
	nrf_delay_ms(50);

	while(true) {
		NRF_LOG_PRINTF("\n\rLIS2HH12 accelerometer library demo\r\n");
		lis2Update();
		NRF_LOG_PRINTF("X acceleration:%d mG\r\n", lis2Accel[0]);
		NRF_LOG_PRINTF("Y acceleration:%d mG\r\n", lis2Accel[1]);
		NRF_LOG_PRINTF("Z acceleration:%d mG\r\n", lis2Accel[2]);
		printf("tilt: %f deg\r\n", lis2Tilt);

		/*
		NRF_LOG_PRINTF("_LIS2_REG_CTRL1 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL1));
		NRF_LOG_PRINTF("_LIS2_REG_CTRL2 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL2));
		NRF_LOG_PRINTF("_LIS2_REG_CTRL3 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL3));
		NRF_LOG_PRINTF("_LIS2_REG_CTRL4 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL4));
		NRF_LOG_PRINTF("_LIS2_REG_CTRL5 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL5));
		NRF_LOG_PRINTF("_LIS2_REG_CTRL6 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL6));
		NRF_LOG_PRINTF("_LIS2_REG_CTRL7 %04x\r\n", _lis2ReadReg(_LIS2_REG_CTRL7));
		*/

		NRF_LOG_PRINTF("\r\n");
		nrf_delay_ms(1000);
	}
}

