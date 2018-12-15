#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"

#include "../../../library/codec/e_sound.h"
#include "../../../library/motor_led/e_init_port.h"
#include "../../../library/motor_led/e_motors.h"
#include "../../../library/motor_led/e_led.h"
#include "../../../library/uart/e_uart_char.h"
#include "../../../library/a_d/e_ad_conv.h"
#include "../../../library/a_d/e_prox.h"
#include "math.h"
#include "utility.h"

int objectfollowing_sensorzero[8];
int objectfollowing_weightleft[8] = {8,8,4,0,0,-6,-12,-12};
int objectfollowing_weightright[8] = {-12,-12,-6,0,0,4,8,8};

void objectfollowing_sensor_calibrate() {
	int i, j;
	char buffer[80];
	long sensor[8];

	for (i=0; i<8; i++) {
		sensor[i]=0;
	}
	
	for (j=0; j<32; j++) {
		for (i=0; i<8; i++) {
			sensor[i]+=e_get_prox(i);
		}
		wait(10000);
	}

	for (i=0; i<8; i++) {
		objectfollowing_sensorzero[i]=(sensor[i]>>5);
		sprintf(buffer, "%d, ", objectfollowing_sensorzero[i]);
		e_send_uart1_char(buffer, strlen(buffer));
	}

	sprintf(buffer, " calibration done\r\n");
	e_send_uart1_char(buffer, strlen(buffer));
	wait(100000);
}

void run_objectfollowing() {
	int i;
	int sensor;
	char buffer[80];
	int leftwheel, rightwheel;

	// Init sensors
	e_init_port();
	e_init_motors();
	e_init_prox();

	// Calibrate sensors
	e_set_led(8, 1);
	objectfollowing_sensor_calibrate();
	e_set_led(8, 0);

	// 
	while (1) {
		leftwheel=200;
		rightwheel=200;
		for (i=0; i<8; i++) {
			sensor=e_get_prox(i)-objectfollowing_sensorzero[i];
			sprintf(buffer, "%d, ", sensor);
			e_send_uart1_char(buffer, strlen(buffer));
			leftwheel+=objectfollowing_weightleft[i]*(sensor>>4);
			rightwheel+=objectfollowing_weightright[i]*(sensor>>4);
		}

		sprintf(buffer, "setspeed %d %d\r\n", leftwheel, rightwheel);
		e_send_uart1_char(buffer, strlen(buffer));
		
		if (leftwheel>800) {leftwheel=800;}
		if (rightwheel>800) {rightwheel=800;}
		if (leftwheel<-800) {leftwheel=-800;}
		if (rightwheel<-800) {rightwheel=-800;}
		e_set_speed_left(leftwheel);
		e_set_speed_right(rightwheel);
		wait(100000);
	}
}

