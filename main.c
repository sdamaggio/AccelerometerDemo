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
#include "app_util_platform.h"
#include "app_error.h"
#include "app_twi.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_error.h"
#include "lis2.h"


/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

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

#define MAX_PENDING_TWI_TRANSACTIONS    4
#define READ_BUFFER_SIZE    			5
#define _LIS2_TEST_TRANSFERS			1

int16_t lis2Accel[3];
float lis2Tilt;
uint8_t readBuffer[READ_BUFFER_SIZE];

uint8_t err_code_test;
static uint8_t const buf[] = {_LIS2_REG_CTRL1, 0};
app_twi_transfer_t const transfers[_LIS2_TEST_TRANSFERS] =
{
	APP_TWI_WRITE(_LIS2_I2CADDR, buf, sizeof(buf), 0),
};

/* TWI instance. */
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TWI_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}

void lis2Init()
{
	twi_config();
}

//static void _lis2ReadRegTran()
//{
//	uint8_t regAddr = _LIS2_REG_WHO_AM_I;
//	//uint8_t res = 0;
//	uint8_t err_code;
//
//	//static uint8_t const buf[] = { LM75B_REG_CONF, 0 };
//
//    app_twi_transfer_t const transfers[] =
//    {
//		APP_TWI_WRITE(_LIS2_I2CADDR, &regAddr, sizeof(regAddr), APP_TWI_NO_STOP),
//		APP_TWI_READ (_LIS2_I2CADDR, readBuffer, READ_BUFFER_SIZE, 0)
//    };
//
////    uint8_t const lm75b_conf_reg_addr  = LM75B_REG_CONF;
////    LM75B_READ(&lm75b_conf_reg_addr,  &m_buffer[0], 1),
////	#define LM75B_READ(p_reg_addr, p_buffer, byte_cnt) APP_TWI_WRITE(LM75B_ADDR, p_reg_addr, 1, APP_TWI_NO_STOP), APP_TWI_READ (LM75B_ADDR, p_buffer, byte_cnt, 0)
//
//    NRF_LOG_PRINTF("app_twi_perform -pre \r\n");
//    nrf_delay_ms(200);
//    // Blocking mode: When an application schedules a transaction in blocking mode (app_twi_perform),
//    // the function does not return until the transaction has finished (successfully or with an error).
//    err_code = app_twi_perform(&m_app_twi, transfers, 2, NULL);
//    NRF_LOG_PRINTF("app_twi_perform %d \r\n", err_code);
//    nrf_delay_ms(500);
//    APP_ERROR_CHECK(err_code);
//}

static void _lis2WriteRegTran()
{
    // Blocking mode: When an application schedules a transaction in blocking mode (app_twi_perform),
    // the function does not return until the transaction has finished (successfully or with an error).
	err_code_test = app_twi_perform(&m_app_twi, transfers, _LIS2_TEST_TRANSFERS, NULL);
}

/**
 * @brief Function for main application entry.
 */
int main(void) {
	bool b = false;
	NRF_LOG_PRINTF("\n\rLIS2HH12 accelerometer library demo\r\n");
	lis2Init();
	nrf_delay_ms(1000);
	//lis2SetOutputDataRate(LIS2_ODR10HZ);
	//nrf_delay_ms(50);

	while(true) {
		b =! b;
		NRF_LOG_PRINTF("__%d__\r\n", b);
		_lis2WriteRegTran();
		NRF_LOG_PRINTF("app_twi_perform() error:%d \r\n", err_code_test);
		nrf_delay_ms(1000);
		APP_ERROR_CHECK(err_code_test);

		/*
		lis2Update();
		NRF_LOG_PRINTF("X acceleration:%d mG\r\n", lis2Accel[LIS2_X]);
		NRF_LOG_PRINTF("Y acceleration:%d mG\r\n", lis2Accel[LIS2_Y]);
		NRF_LOG_PRINTF("Z acceleration:%d mG\r\n", lis2Accel[LIS2_Z]);
		printf("tilt: %f deg\r\n", lis2Tilt);
		*/
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
		nrf_delay_ms(3000);
	}
}

