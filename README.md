# MYSENTINEL 工程说明

## 工程简介
本工程基于 STM32F427 芯片，采用 FreeRTOS 实时操作系统，集成了 CAN 通信、IMU 姿态解算、遥控器输入、底盘与三脚架控制等功能，适用于机器人或运动控制类项目。

## 主要功能模块

- **CAN 通信**：支持 CAN1、CAN2 双路通信，负责与外部设备（如电机、传感器等）进行数据交互。
- **IMU 姿态解算**：集成 MPU6500 陀螺仪与 IST8310 磁力计，完成原始数据采集、零偏校准、四元数与欧拉角姿态解算。
- **遥控器输入**：通过 USART3 + DMA 实现遥控器（如 DBUS）信号的高效接收。
- **底盘与三脚架控制**：实现底盘运动与三脚架机构的控制逻辑。
- **FreeRTOS 多任务调度**：各功能模块通过任务和中断协同运行，提升系统实时性与可靠性。

## 主要文件说明

- `Core/Inc/` & `Core/Src/`
  - `main.c`：主程序入口，完成外设初始化、RTOS 启动等。
  - `can.h/can.c`：CAN 总线初始化与数据收发。
  - `dma.h/dma.c`：DMA 控制器配置。
  - `spi.h/spi.c`：SPI 总线配置，主要用于 IMU 通信。
  - `usart.h/usart.c`：串口配置，遥控器输入等。
  - `freertos.c`：FreeRTOS 任务与调度相关代码。
  - `gpio.h/gpio.c`：GPIO 管脚配置。

- `MDK-ARM/`
  - `bsp_imu.h/bsp_imu.c`：IMU（MPU6500+IST8310）驱动与姿态解算。
  - `bsp_imu.h`：对外接口及数据结构定义。
  - `bsp_imu.c`：IMU 初始化、数据采集、四元数与欧拉角解算实现。
  - `bsp_imu.c` 需配合 SPI5、GPIOF.6（NSS）等硬件资源。

- `user/application/`
  - `canbus.h/canbus.c`：CAN 通信协议封装。
  - `rc.h/rc.c`：遥控器输入解析。
  - `pid.h/pid.c`：PID 控制算法。
  - `tripod.c`：云台控制逻辑。
  - `underpan.c`：底盘控制逻辑。

- `Drivers/`：芯片厂商提供的 HAL、CMSIS、外设驱动库。
- `Middlewares/`：第三方中间件（如 FreeRTOS）。

## 使用说明
1. 使用 Keil、STM32CubeIDE 或其他支持 STM32F4 的开发环境打开本工程。
2. 确认硬件连接无误（CAN、IMU、遥控器等）。
3. 编译并下载程序到目标板。
4. 通过串口、CAN 或调试接口进行功能验证。

## 维护者
- 作者：DJI Robotics
- 日期：2018-2026
