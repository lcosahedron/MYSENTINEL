/*******************************************************************************
 * @file    imu.c
 * @date    2026/4/15
 * @brief   IMU数据获取和处理，使用mpu6050+mag3110的组合，mpu6050提供加速度和陀螺仪数据，
 * mag3110提供磁力计数据，使用互补滤波进行姿态解算，提供四元数和欧拉角两种数据格式，单位分别为无量纲和°，
 * 可以通过调用imu_ahrs_update()函数进行姿态解算，调用imu_attitude_update()函数将四元数转换为欧拉角，
 * 调用mpu_offset_call()函数进行陀螺 仪零偏校准，调用mpu_get_data()函数获取最新的传感器数据并进行姿态解算，
 * 获取的数据保存在imu_t类型的imu变量中，包含了加速度、陀螺仪、磁力计数据以及姿态数据，具体数据结构详见bsp_imu.h
 ******************************************************************************/ 
#include "bsp_imu.h"
#include "cmsis_os.h"

void StartGetIMUData(void *argument)
{
  /* USER CODE BEGIN StartGetIMUData */
  /* Infinite loop */
  for(;;)
  {
    mpu_get_data();
	imu_ahrs_update();
	imu_attitude_update();
    osDelay(10);
  }
  /* USER CODE END StartGetIMUData */
}
