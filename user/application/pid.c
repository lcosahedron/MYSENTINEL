#include "pid.h"
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->previous_error = 0.0f;
}
float PID_Compute(PID_Controller *pid, float input){
    pid->error = pid->setpoint - input;
    pid->integral += pid->error;
    pid->derivative = pid->error - pid->previous_error;
    pid->previous_error = pid->error;

    return (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * pid->derivative);
}