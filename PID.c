/*
	PID controller

	By Victor Henrique Salvi (victorsvi@gmail.com)

	
	
	
	Implements a Proportional Integrative Derivative controller. It calculates a output based on a input and some parameters in order to bring the input parameter close to the defined set point.

	The function is the following:
		output = (Kp * error) + (Ki * error * (time - previoustime)) + (Kd * (error - previouserror) / (time - previoustime))
	Where:
		Kp = Proportional constant, in units of (OutUnit / InUnit)
		Ki = Integrative constant, in units of (OutUnit / (InUnit * TimeUnit)
		Kd = Derivative constant, in units of (OutUnit * TimeUnit) / InUnit
		output = response of the system, in units of OutUnit
		error = difference between the input value and the set point, in units of InUnit
		time = current time of the system, in units of TimeUnit

	Its recommended to scale the input and output to get better resolution. If the input or output is scaled, the other parameters should also be scaled. ex: if the input is in the range of 0°C to 100°C, the input can be scaled from 0 to 65,536

	
	
	
	How to use:

	1) The main file should define the constant PID_MAX_CHANNELS, that define the maximum number of systems controlled. The channel will isolate the different systems and will identify them. The channel of a PID must be a number between 1 and PID_MAX_CHANNELS
	2) Call the PID_setConf to set the PID parameters to a particular channel. This function can also be used to update the parameters.
	3) Call PID_Calculate to get the output for a channel. This function will be called multiple times to keep the system stable.

	
	
	
	Release Notes:

		10/11/2018:
			* Frist implementation

*/

#include <stdio.h>
#include "PID.h"


typedef struct {
	//int channel; 	// Channel of the PID. The channel is a number between 1 and PID_MAX_CHANNELS. As the PID needs to remember the last error value and the accumulated error (for the Derivative and Integrative), the channel will insulate multiple PIDs
	float Kp; 		// Proportional gain of the PID
	float Ki;		// Integrative gain of the PID
	float Kd;		// Derivative gain of the PID
} PID_Conf_T;

typedef struct {
	float previousError;
	float accumulatedError;

	unsigned long previousTime;
	PID_Conf_T PID_Conf;
} PID_Memory_T;

//As the PID needs to remember the last error value and the accumulated error (for the Derivative and Integrative), the channel will insulate multiple PIDs
PID_Memory_T PID_Memory[PID_MAX_CHANNELS]; //Supports PID_MAX_CHANNELS channels


/**
	Sets up or updates the PID parameters for a system identified by a channel number.

	@param channel The channel configured
	@param Kp Proportional constant, in units of (OutUnit / InUnit)
	@param Ki Integrative constant, in units of (OutUnit / (InUnit * TimeUnit)
	@param Kd Derivative constant, in units of (OutUnit * TimeUnit) / InUnit
	@returns 0 for success; 1 for error
*/
int PID_setConf (int channel, float Kp, float Ki, float Kd) {

	if(channel <= 0 || channel > PID_MAX_CHANNELS){ // Verify if the channel is a valid number to access the PID_Memory array
		fputs("The PID channel is not valid. Check if the PID_MAX_CHANNELS is set", stderr);
		return 1; // Error status
	}

	//PID_Memory_T[channel - 1].PID_Conf->channel = channel;
	PID_Memory[channel - 1].PID_Conf.Kp = Kp;
	PID_Memory[channel - 1].PID_Conf.Ki = Ki;
	PID_Memory[channel - 1].PID_Conf.Kd = Kd;

	//PID_reset(channel);

	return 0; // Success status
} //PID_setConf

/**
	Resets the PID previous values (for the error and the derivative and integrative terms) for a system identified by a channel number.

	@param channel The channel to be reset
	@returns 0 for success; 1 for error
*/
int PID_reset(int channel) {

	if(channel <= 0 || channel > PID_MAX_CHANNELS){ // Verify if the channel is a valid number to access the PID_Memory array
		fputs("The PID channel is not valid. Check if the PID_MAX_CHANNELS is set", stderr);
		return 1; // Error status
	}

	// Resets the memory of the current channel
	PID_Memory[channel - 1].previousError = 0;
	PID_Memory[channel - 1].accumulatedError = 0;
	PID_Memory[channel - 1].previousTime = 0;

	return 0; // Success status
} //PID_reset

/**
	Calculates the output for a system identified by a channel number.

	@param channel The channel to calculate
	@param input The input is the value of the measured variable of the system, in units of InUnit
	@param setPoint The value that the measured variable should be, in units of InUnit
	@param currTime The current time of the system, in units of TimeUnit. Its the time relative to a fixed value, like the program start
	@returns The output of the controller, in units of OutUnit
*/
float PID_Calculate (int channel, float input, float setPoint, unsigned long currTime) {

	//unsigned long currTime;
	unsigned long diffTime;
	float currError;
	float proportionalTerm;
	float integrativeTerm;
  float integrativeError;
	float derivativeTerm;
	float output;

	if(channel <= 0 || channel > PID_MAX_CHANNELS){ // Verify if the channel is a valid number to access the PID_Memory array
		fputs("The PID channel is not valid. Check if the PID_MAX_CHANNELS is set", stderr);
		return 0;
	}
 
	diffTime = currTime - PID_Memory[channel - 1].previousTime;
	// The time is returned in milisseconds. The long variable will wrap and start from 0 after some time. Thus that frist cycle will "ignore" the integrative and derivative terms in order to avoid erros.
//	if(currTime < PID_Memory[channel - 1].previousTime){
//		diffTime = 0;
//	}
	PID_Memory[channel - 1].previousTime = currTime;

// calculate error
	currError = setPoint - input;

// set proportional term
	proportionalTerm = currError * PID_Memory[channel - 1].PID_Conf.Kp;

// set integrative term
  integrativeError = currError * (float) diffTime;
	PID_Memory[channel - 1].accumulatedError += integrativeError;
	integrativeTerm = PID_Memory[channel - 1].accumulatedError * PID_Memory[channel - 1].PID_Conf.Ki;

// set derivative term
	if(diffTime != 0){
		derivativeTerm = (currError - PID_Memory[channel - 1].previousError) * PID_Memory[channel - 1].PID_Conf.Kd / diffTime;
	}
	else {
		derivativeTerm = 0;
	}
	PID_Memory[channel - 1].previousError = currError;

// set output
	output = proportionalTerm + integrativeTerm + derivativeTerm;

	return output;

} //PID_Calculate


//int main () {}
