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
{              	////线性加速度单位为mg，范围-32768~32767，对应-16g~+16g
	int16_t ax;	//垂直于长边，以标志摆正观察，左负右正
	int16_t ay;	//垂直于短边，加速度向前为正
	int16_t az;	//加速度向下为正
				//磁场强度单位为mGauss，范围-32768~32767，对应-4912mGauss~+4912mGauss
	int16_t mx;
	int16_t my;
	int16_t mz;

	float temp;	//温度，单位为摄氏度
			   	////陀螺仪角速度单位为°/s，范围-32768~32767，对应-2000°/s~+2000°/s
	float wx;  	//沿短轴旋转，从标志方向右视顺时针为正，单位为°/s
	float wy;  	//沿长轴旋转，从标志方向正视逆时针为正，单位为°/s
	float wz;  	//水平旋转，顺时针为正，单位为°/s
			  	
	float vx;
	float vy;
	float vz;
				////姿态角单位为°，范围-180~+180
	float rol;  //沿短轴旋转，从标志方向右视顺时针（低头）为正，单位为°
	float pit;  //沿长轴旋转，从标志方向正视顺时针为正，单位为°
	float yaw;  //水平旋转，顺时针为正，单位为°
} imu_t;		//直接调这个结构体就行，里面包含了mpu数据和姿态数据

extern mpu_data_t mpu_data;
extern imu_t      imu;

uint8_t   mpu_device_init(void);
void init_quaternion(void);
void mpu_get_data(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
void mpu_offset_call(void);

#endif


