#ifndef PID_H
#define PID_H

// PID controller structure
typedef struct {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
    float setpoint;  // 目标值
    float error;     // Current error
    float integral;  // 积分项
    float derivative; // 微分项
    float previous_error;  // 前次误差，用于计算微分项
} PID_Controller;

// Function prototypes
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Compute(PID_Controller *pid, float input);

#endif // PID_H