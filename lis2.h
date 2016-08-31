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

#ifndef LIS2_H_
#define LIS2_H_

/**
 * @file
 *
 * @ingroup        I2CDrivers
 *
 * Provides asynchronous access to the LIS2HH2 I2C 3-axis accelerometer.
 *
 * After initializing call lis2Update to initiate a new conversion.
 * Use lis2Accel to read the newly acquired sensor data.
 *
 * We always use 2G mode.
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Enumerator which defines output data rates available for the LIS2HH12 accelerometer.
 */
typedef enum _Lis2OutputDataRate
{
	LIS2_POWERDOWN,
	LIS2_ODR10HZ,
	LIS2_ODR50HZ,
	LIS2_ODR100HZ,
	LIS2_ODR200HZ,
	LIS2_ODR400HZ,
	LIS2_ODR800HZ,
} Lis2OutputDataRate;


#define LIS2_X    0        //!< Index of X value for the lis2Accel array.
#define LIS2_Y    1        //!< Index of Y value for the lis2Accel array.
#define LIS2_Z    2        //!< Index of Z value for the lis2Accel array.


/*
 * The last X, Y and Z values. Each item is 10, two's complement notation, running from -32738 to
 * +32737, which translates to -2g to ~ +2g. The resolution is therefore is 4G / (2^16 - 1), i.e. 0.061mG
 * per step.
 */
extern int16_t lis2Accel[3];



/**
 * @brief Function to initialize the lis2 accelerometer
 *
 * Default initialization includes +/-2g sensitivity, odr set to LIS2_POWERDOWN,
 * FIFO set to Bypass mode, filtering and interrupts disabled
 */
void lis2Init();



/**
 * @brief Function to set lis2 output data rate
 *
 * Set by default to LIS2_POWERDOWN.
 * The higher the frequency the higher the power consumption.
 *
 * @param odr selected Output Data Rate from available odr in enum Lis2OutputDataRate
 */
bool lis2SetOutputDataRate(Lis2OutputDataRate odr);



/**
 * @brief Function to update lis2 x, y and z acceleration readings
 */
void lis2Update();



/**
 * @brief Function to update lis2 temperature reading
 *
 * WARNING: value seems to not make sense!
 * Temperature value is stored in 2 8bit registers, 16 bit total
 * Resolution is 11 bit expressed in two's complement
 * Range is from -40 to +85 °C so 125 total, 0 is at ((40+85)/2)+40=22.5°C
 * Ratio is then 0.061035°C/bit
 * Tout = (int16_t)value * ratio + 22.5°C
 *
 * @return	int16_t temperature register value
 */
int16_t lis2GetTemp();

#endif /* LIS2_H_ */
