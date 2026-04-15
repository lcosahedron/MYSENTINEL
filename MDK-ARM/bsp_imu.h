/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__

#include "mytype.h"
#define MPU_DELAY(x) HAL_Delay(x)

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float temp;

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu;

/**
 * @brief Initialize MPU6500 and IST8310 devices and run offset calibration.
 * @return 0 when initialization is successful, non-zero on failure.
 */
uint8_t   mpu_device_init(void);

/**
 * @brief Initialize AHRS quaternion using current magnetic field direction.
 */
void init_quaternion(void);

/**
 * @brief Read one frame of raw IMU data and update scaled gyro values.
 */
void mpu_get_data(void);

/**
 * @brief Run one AHRS update step based on current accelerometer/gyro/magnetometer data.
 */
void imu_ahrs_update(void);

/**
 * @brief Convert quaternion to Euler angles (yaw, pitch, roll) in degrees.
 */
void imu_attitude_update(void);

/**
 * @brief Calculate gyroscope and accelerometer zero offsets.
 */
void mpu_offset_call(void);

#endif


