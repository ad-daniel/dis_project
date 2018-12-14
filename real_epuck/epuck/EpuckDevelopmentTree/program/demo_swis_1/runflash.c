#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"

#include "../../../library/motor_led/e_init_port.h"
#include "../../../library/a_d/e_accelerometer.h"
#include "../../../library/motor_led/e_led.h"
#include "../../../library/uart/e_uart_char.h"
#include "../../../library/a_d/e_ad_conv.h"
#include "../../../library/a_d/e_prox.h"
#include "utility.h"

void run_flash() {
	int i;
	int sensor;
	int sensor_min, sensor_max;
	char buffer[80];

	// Initialize
	e_init_prox();
	e_set_led(8, 0);

	while (1) {
		// Wait for flash
		while (1) {
			sensor_min=4095;
			sensor_max=0;
			for (i=0; i<30000; i++) {
				sensor=e_get_ambient_light(0);
				if (sensor<100) {
					e_set_led(8, 1);
				}

				if (sensor_min>sensor) {sensor_min=sensor;}
				if (sensor_max<sensor) {sensor_max=sensor;}
			}

			sprintf(buffer, "%d %d\r\n", sensor_min, sensor_max);
			e_send_uart1_char(buffer, strlen(buffer));

			if (sensor_min<1000) {
				break;
			}
		}

		// Wait for some constant time and then reset the leds
		wait(10000);
		e_set_led(8, 0);
	}
}
