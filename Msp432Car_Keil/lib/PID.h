#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	/* 控制器增益 Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* 微分低通滤波器时间常数 Derivative low-pass filter time constant */
	float tau;

	/* 输出限制 Output limits */
	float limMin;
	float limMax;
	
	/* 积分器限制 Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* 采样时间（以秒为单位） Sample time (in seconds) */
	float T;

	/* 控制器“记忆” Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* 控制器输出 Controller output */
	float out;

} PID;

void  PID_Init(PID *pid);
float PID_Update(PID *pid, float setpoint, float measurement);

#endif
