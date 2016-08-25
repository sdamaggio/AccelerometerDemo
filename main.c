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
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "app_util_platform.h"




/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

/*Common addresses definition for accelereomter. */
#define MMA8451_ADDR        (0x1CU >> 1) // 7bit I2C address, SA0 pin is to GND

#define MMA8451_REG_STATUS				0x00U
#define MMA8451_REG_OUT_X_MSB			0x01U
#define MMA8451_REG_OUT_X_LSB			0x02U
#define MMA8451_REG_OUT_Y_MSB			0x03U
#define MMA8451_REG_OUT_Y_LSB			0x04U
#define MMA8451_REG_OUT_Z_MSB			0x05U
#define MMA8451_REG_OUT_Z_LSB			0x06U
#define MMA8451_REG_F_SETUP				0x09U
#define MMA8451_REG_TRIG_CFG			0x0AU
#define MMA8451_REG_SYSMOD				0x0BU
#define MMA8451_REG_INT_SOURCE			0x0CU
#define MMA8451_REG_WHO_AM_I			0x0DU
#define MMA8451_REG_XYZ_DATA_CFG		0x0EU
#define MMA8451_REG_HP_FILTER_CUTOFF	0x0FU
#define MMA8451_REG_PL_STATUS			0x10U
#define MMA8451_REG_PL_CFG				0x11U
#define MMA8451_REG_PL_COUNT			0x12U
#define MMA8451_REG_PL_BF_ZCOMP			0x13U
#define MMA8451_REG_P_L_THS_REG			0x14U
#define MMA8451_REG_FF_MT_CFG			0x15U
#define MMA8451_REG_FF_MT_SRC			0x16U
#define MMA8451_REG_FF_MT_THS			0x17U
#define MMA8451_REG_FF_MT_COUNT			0x18U
#define MMA8451_REG_TRANSIENT_CFG		0x1DU
#define MMA8451_REG_TRANSIENT_SCR		0x1EU
#define MMA8451_REG_TRANSIENT_THS		0x1FU
#define MMA8451_REG_TRANSIENT_COUNT		0x20U
#define MMA8451_REG_PULSE_CFG			0x21U
#define MMA8451_REG_PULSE_SRC			0x22U
#define MMA8451_REG_PULSE_THSX			0x23U
#define MMA8451_REG_PULSE_THSY			0x24U
#define MMA8451_REG_PULSE_THSZ			0x25U
#define MMA8451_REG_PULSE_TMLT			0x26U
#define MMA8451_REG_PULSE_LTCY			0x27U
#define MMA8451_REG_PULSE_WIND			0x28U
#define MMA8451_REG_ASLP_COUNT			0x29U
#define MMA8451_REG_CTRL_REG1			0x2AU
#define MMA8451_REG_CTRL_REG2			0x2BU
#define MMA8451_REG_CTRL_REG3			0x2CU
#define MMA8451_REG_CTRL_REG4			0x2DU
#define MMA8451_REG_CTRL_REG5			0x2EU
#define MMA8451_REG_OFF_X				0x2FU
#define MMA8451_REG_OFF_Y				0x30U
#define MMA8451_REG_OFF_Z				0x31U


/* Mode for MMA8451. */
#define ACTIVE_MODE 1u

/*Failure flag for reading from accelerometer. */
#define MMA8451_FAILURE_FLAG (1u << 6)

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
static sum_t m_sum = {0};

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
static sample_t m_sample_buffer[NUMBER_OF_SAMPLES] = {0};
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
static const nrf_drv_twi_t m_twi_mma_8451 = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief Function for casting 6 bit uint to 6 bit int.
 *
 */
__STATIC_INLINE void int_to_uint(int8_t * put, uint8_t data)
{
    if (!(data & MMA8451_FAILURE_FLAG))     //6th bit is failure flag - we cannot read sample
    {
        *put = (int8_t)(data << 2) / 4;
    }
}

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
 * @brief Function for setting active mode on MMA8451 accelerometer.
 */
void MMA8451_set_mode(void)
{
    /*ret_code_t err_code;

    uint8_t reg[2] = {MMA8451_REG_MODE, ACTIVE_MODE};

    err_code = nrf_drv_twi_tx(&m_twi_mma_8451, (MMA8451_ADDR << 1 | 0), reg, sizeof(reg), false); // write accelerometer address with R/W bit set to 0 (write)
    APP_ERROR_CHECK(err_code);

    while(m_set_mode_done == false);*/
}

/**
 * @brief Function for averaging samples from accelerometer.
 */
void read_data(sample_t * p_new_sample)
{
    /* Variable to count samples. */
    static uint8_t sample_idx;
    static uint8_t prev_tilt;

    sample_t * p_sample = &m_sample_buffer[sample_idx];

    /* Subtracting oldest sample. */
    m_sum.x    -= p_sample->x.conv;
    m_sum.y    -= p_sample->y.conv;
    m_sum.z    -= p_sample->z.conv;

    p_sample->tilt = p_new_sample->tilt;

    int_to_uint(&p_sample->x.conv, p_new_sample->x.raw);
    int_to_uint(&p_sample->y.conv, p_new_sample->y.raw);
    int_to_uint(&p_sample->z.conv, p_new_sample->z.raw);

    /* Adding new sample. This way we always have defined number of samples. */
    m_sum.x    += p_sample->x.conv;
    m_sum.y    += p_sample->y.conv;
    m_sum.z    += p_sample->z.conv;

    ++sample_idx;
    if (sample_idx >= NUMBER_OF_SAMPLES)
    {
        sample_idx = 0;
    }

    if (sample_idx == 0 || (prev_tilt && (prev_tilt != p_sample->tilt)))
    {
        char const * orientation;
        switch ((p_sample->tilt >> 2) & 0x07)
        {
            case LEFT:
                orientation = "LEFT";
                break;
            case RIGHT:
                orientation = "RIGHT";
                break;
            case DOWN:
                orientation = "DOWN";
                break;
            case UP:
                orientation = "UP";
                break;
            default:
                orientation = "?";
                break;
        }
        printf("X: %3d, Y: %3d, Z: %3d | %s%s%s\r\n",
                m_sum.x / NUMBER_OF_SAMPLES,
                m_sum.y / NUMBER_OF_SAMPLES,
                m_sum.z / NUMBER_OF_SAMPLES,
                orientation,
                (p_sample->tilt & TILT_TAP_MASK) ? " TAP"   : "",
                (p_sample->tilt & TILT_SHAKE_MASK) ? " SHAKE" : "");
                prev_tilt = p_sample->tilt;
    }
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    // static sample_t m_sample;
    static uint8_t reading = 0;
    
    NRF_LOG_PRINTF("-handler started: %d\n", p_event->type);

    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
        	NRF_LOG_PRINTF("-case NRF_DRV_TWI_EVT_DONE:\n");
            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) && (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX))
            {
                if(m_set_mode_done != true)
                {
                    m_set_mode_done  = true;
                    return;
                }
                m_xfer_done = false;
                /* Read 4 bytes from the specified address. */
                err_code = nrf_drv_twi_rx(&m_twi_mma_8451, MMA8451_ADDR, &reading, sizeof(reading));
                APP_ERROR_CHECK(err_code);
                NRF_LOG_PRINTF("-WHO_AM_I: %d right after\n", reading);
            }
            else
            {
                //read_data(&m_sample);
            	NRF_LOG_PRINTF("-WHO_AM_I: %d\n", reading);

                m_xfer_done = true;
            }
            break;

        default:
        	NRF_LOG_PRINTF("-case default event type:%d\n", p_event->type);
            break;        
    }   
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_mma_8451_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_mma_8451, &twi_mma_8451_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_PRINTF("twi init: %d\n", err_code);
    
    nrf_drv_twi_enable(&m_twi_mma_8451);
}

/**
 * @brief Function for main application entry.
 */
/*int main(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT());
	NRF_LOG_PRINTF("\n\nSTARTING\n\n\n");

    uart_config();
   // int a = __GNUC__, c = __GNUC_PATCHLEVEL__;
    //NRF_LOG_PRINTF("\n\rTWI sensor example\r\n");
    nrf_gpio_cfg_output(BSP_LED_0);




    twi_init();
    //MMA8451_set_mode();

    // i2c start condition: MMA8451 address with write bit set to 0 (write)

    // read register: send register address and R/W bit set to 1 (read)

    
    uint8_t reg = MMA8451_REG_WHO_AM_I;
    ret_code_t err_code;
    
    while(true)
    {
    	NRF_LOG_PRINTF("____\r\n");

        // Start transaction with a slave with the specified address.



        err_code = nrf_drv_twi_tx(&m_twi_mma_8451, MMA8451_ADDR, &reg, sizeof(reg), true);
        NRF_LOG_PRINTF("request sent\r\n");
        APP_ERROR_CHECK(err_code);


        do
        {
			NRF_LOG_PRINTF("waiting...\r\n");
			__WFE();
			NRF_LOG_PRINTF("...done waiting\r\n");
		} while(m_xfer_done == false);
        m_xfer_done = false;

        nrf_delay_ms(1000);
		nrf_gpio_pin_clear(BSP_LED_0);
		nrf_delay_ms(1000);
		nrf_gpio_pin_set(BSP_LED_0);
    }
    nrf_delay_ms(500);
}*/

int main(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT());
	NRF_LOG_PRINTF("\n\nSTARTING\n\n\n");
    nrf_gpio_cfg_output(BSP_LED_0);


    uart_config();
    twi_init();

    while(true)
    {
    	NRF_LOG_PRINTF("____\r\n");

    	ret_code_t err_code;
		static uint8_t reading = 1;


		err_code = nrf_drv_twi_tx(&m_twi_mma_8451, MMA8451_ADDR, MMA8451_REG_WHO_AM_I, 7, true);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_PRINTF("tx with WHO_AM_I address and value 1; err_code: %d data: %d\n", err_code, reading);

		do
		{
			NRF_LOG_PRINTF("wait...\r\n");
			__WFE();
			NRF_LOG_PRINTF("done waiting\r\n");
		}while(m_xfer_done == false);

		m_xfer_done = false;

		//

		// blink the led
        nrf_delay_ms(1000);
		nrf_gpio_pin_clear(BSP_LED_0);
		nrf_delay_ms(1000);
		nrf_gpio_pin_set(BSP_LED_0);
		nrf_delay_ms(500);
    }




}

/** @} */
