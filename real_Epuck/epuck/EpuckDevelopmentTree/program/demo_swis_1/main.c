#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#include "codec/e_sound.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"
#include "uart/e_uart_char.h"
#include "a_d/e_accelerometer.h"
#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"
#include "I2C/e_I2C_protocol.h"
#include "contrib/radio_swis/e_radio_swis.h"
#include "contrib/robot_id/e_robot_id.h"

#include "runcollaboration.h"
#include "runaccelerometer.h"
#include "runbraitenberg.h"
#include "runobjectfollowing.h"
#include "runlocatesound.h"
#include "runwallfollow.h"
#include "runacclive.h"
#include "runflash.h"


#define PI 3.14159265358979

int main() {
	char buffer[80];
	int selector;

	//system initialization 
	e_init_port();
	e_init_uart1();

	//Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR=0;
		__asm__ volatile ("reset");
	}

	// Decide upon program
	selector = e_get_selector();
	if (selector==0) {
		sprintf(buffer, "Program %d: run_accelerometer\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_accelerometer();
	} else if (selector==1) {
		sprintf(buffer, "Program %d: run_locatesound\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_locatesound();
	} else if (selector==2) {
		sprintf(buffer, "Program %d: run_wallfollow\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_wallfollow();
	} else if (selector==3) {
		sprintf(buffer, "Program %d: run_flash\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_flash();
	} else if (selector==4) {
		sprintf(buffer, "Program %d: run_objectfollowing\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_objectfollowing();
	} else if ((selector==11) || (selector==12) || (selector==13)) {
		sprintf(buffer, "Program %d: run_collaboration\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_collaboration();
	} else {
		sprintf(buffer, "Program %d: run_braitenberg\r\n", selector);
		e_send_uart1_char(buffer, strlen(buffer));
		run_braitenberg();
	}

	while(1);
	return 0;
}

