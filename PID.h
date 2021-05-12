#ifndef PID_H
#define PID_H

#ifdef __cplusplus
	extern "C" {
#endif




#ifndef PID_MAX_CHANNELS
#define PID_MAX_CHANNELS 1 //
#endif

int PID_setConf (int channel, float Kp, float Ki, float Kd);

int PID_reset(int channel);

float PID_Calculate (int channel, float input, float setPoint, unsigned long currTime);




#ifdef __cplusplus
	}
#endif

#endif
