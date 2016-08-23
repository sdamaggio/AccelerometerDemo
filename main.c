/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;


/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

/*Common addresses definition for accelereometer LIS2HH12. */
#define LIS2HH12_ADDR				0b0011101	// 7bit I2C address

#define LIS2HH12_REG_TEMP_L			0x0B
#define LIS2HH12_REG_TEMP_H			0x0C
#define LIS2HH12_REG_WHO_AM_I		0x0F
#define LIS2HH12_REG_ACT_THS		0x1E
#define LIS2HH12_REG_ACT_DUR		0x1F
#define LIS2HH12_REG_CTRL1			0x20
#define LIS2HH12_REG_CTRL2			0x21
#define LIS2HH12_REG_CTRL3			0x22
#define LIS2HH12_REG_CTRL4			0x23
#define LIS2HH12_REG_CTRL5			0x24
#define LIS2HH12_REG_CTRL6			0x25
#define LIS2HH12_REG_CTRL7			0x26
#define LIS2HH12_REG_STATUS			0x27
#define LIS2HH12_REG_OUT_X_L		0x28
#define LIS2HH12_REG_OUT_X_H		0x29
#define LIS2HH12_REG_OUT_Y_L		0x2A
#define LIS2HH12_REG_OUT_Y_H		0x2B
#define LIS2HH12_REG_OUT_Z_L		0x2C
#define LIS2HH12_REG_OUT_Z_H		0x2D
#define LIS2HH12_REG_FIFO_CTRL		0x2E
#define LIS2HH12_REG_FIFO_SRC		0x2F
#define LIS2HH12_REG_IG_CFG1		0x30
#define LIS2HH12_REG_IG_SRC1		0x31
#define LIS2HH12_REG_IG_THS_X1		0x32
#define LIS2HH12_REG_IG_THS_Y1		0x33
#define LIS2HH12_REG_IG_THS_Z1		0x34
#define LIS2HH12_REG_IG_DUR1		0x35
#define LIS2HH12_REG_IG_CFG2		0x36
#define LIS2HH12_REG_SRC2			0x37
#define LIS2HH12_REG_IG_THS2		0x38
#define LIS2HH12_REG_IG_DUR2		0x39
#define LIS2HH12_REG_XL_REFERENCE	0x3A
#define LIS2HH12_REG_XH_REFERENCE	0x3B
#define LIS2HH12_REG_YL_REFERENCE	0x3C
#define LIS2HH12_REG_YH_REFERENCE	0x3D
#define LIS2HH12_REG_ZL_REFERENCE	0x3E
#define LIS2HH12_REG_ZH_REFERENCE	0x3F

#define LIS2HH12_REG_FIFO_stream		01001000b	// to set fifo in Stream mode, just for testing



/* Mode for MMA7660. */
#define ACTIVE_MODE 1u

/*Failure flag for reading from accelerometer. */
#define MMA7660_FAILURE_FLAG (1u << 6)

/*Tilt specific bits*/
#define TILT_TAP_MASK (1U << 5)
#define TILT_SHAKE_MASK (1U << 7)

// [max 255, otherwise "int16_t" won't be sufficient to hold the sum
//  of accelerometer samples]
#define NUMBER_OF_SAMPLES 20

/* Define version of GCC. */
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)

/**
 * @brief Structure for holding sum of samples from accelerometer.
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;
//static sum_t m_sum = {0};

/**
 * @brief Union to keep raw and converted data from accelerometer samples at one memory space.
 */
typedef union{
    uint8_t raw;
    int8_t  conv;
}elem_t;

/**
 * @brief Enum for selecting accelerometer orientation.
 */
typedef enum{
    LEFT = 1,
    RIGHT = 2,
    DOWN = 5,
    UP = 6
}accelerometer_orientation_t;

/**
 * @brief Structure for holding samples from accelerometer.
 */
typedef struct
{
    elem_t  x;
    elem_t  y;
    elem_t  z;
    uint8_t tilt;
} sample_t;

#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"           // Hack to GCC 4.9.3 bug. Can be deleted after switch on using GCC 5.0.0
#endif
#endif
/* Buffer for samples. */
//static sample_t m_sample_buffer[NUMBER_OF_SAMPLES] = {0};
#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic pop
#endif
#endif
/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;
/* TWI instance. */
static const nrf_drv_twi_t m_twi_LIS2HH12 = NRF_DRV_TWI_INSTANCE(0);



/**
 * @brief UART events handler.
 */
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
/*void LIS2HH12_set_mode(void)
{
    ret_code_t err_code;
    // Writing to MMA7660_REG_MODE "1" enables the accelerometer.
    uint8_t reg[2] = {MMA7660_REG_MODE, ACTIVE_MODE};

    err_code = nrf_drv_twi_tx(&m_twi_LIS2HH12, LIS2HH12_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    
    while(m_set_mode_done == false);
}*/

/**
 * @brief Function for checking the WHO_AM_I register to test I2C connection
 */
void LIS2HH12_I2Ctest(void)
{
    /*ret_code_t err_code;

    uint8_t reg[2] = {MMA7660_REG_MODE, ACTIVE_MODE};

    err_code = nrf_drv_twi_tx(&m_twi_LIS2HH12, LIS2HH12_ADDR, LIS2HH12_REG_WHO_AM_I, sizeof(LIS2HH12_REG_WHO_AM_I), false);
    APP_ERROR_CHECK(err_code);

    while(m_set_mode_done == false);
    */
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    static sample_t m_sample;
    
    printf("\n\revent generated on I2C!\r\n");

    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) && (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX))
            {
                if(m_set_mode_done != true)
                {
                    m_set_mode_done  = true;
                    return;
                }
                m_xfer_done = false;
                /* Read 4 bytes from the specified address. */
                err_code = nrf_drv_twi_rx(&m_twi_LIS2HH12, LIS2HH12_ADDR, (uint8_t*)&m_sample, sizeof(m_sample));
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                //read_data(&m_sample);
                m_xfer_done = true;
            }
            break;
        default:
            break;        
    }   
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_LIS2HH12_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_LIS2HH12, &twi_LIS2HH12_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_LIS2HH12);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
	// Configure LED-pins as outputs for debugging.
	LEDS_CONFIGURE(LEDS_MASK);

    uart_config();
    // int a = __GNUC__, c = __GNUC_PATCHLEVEL__;
    printf("\n\rLIS2HH12 sensor example\r\n");
    twi_init();
    //LIS2HH12_set_mode();
    
    uint8_t reg = 0;
    ret_code_t err_code;
    

    while(true)
    {
        nrf_delay_ms(100);
        // Start transaction with a slave with the specified address.
        do
        {
            __WFE();
        }while(m_xfer_done == false);
        err_code = nrf_drv_twi_tx(&m_twi_LIS2HH12, LIS2HH12_ADDR, &reg, sizeof(reg), true);
        APP_ERROR_CHECK(err_code);
        m_xfer_done = false;
    }
}

/** @} */
