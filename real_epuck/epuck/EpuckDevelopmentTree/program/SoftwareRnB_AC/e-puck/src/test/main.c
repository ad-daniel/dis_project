
// simple test :  send numbers

#include <e_ad_conv.h>
#include <e_init_port.h>
#include <e_epuck_ports.h>
#include <e_uart_char.h>
#include <e_led.h>

#include <e_led.h>
#include <e_motors.h>
#include <e_agenda.h>

#include <stdio.h>
#include <string.h>
#include <ircom.h>
#include <btcom.h>
#include <math.h>

float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void obstacleAvoidance();

int main()
{
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();

    // rely on selector to define the role
    int selector = getselector();

    // print in minicom
    char buffer[80];
    sprintf(buffer, "Starting with selector pos %d\r\n", selector);
    e_send_uart1_char(buffer, strlen(buffer));

    // wait for s to start
    btcomWaitForCommand('s');
    btcomSendString("==== READY - IR TESTING ====\n\n");

    e_calibrate_ir(); 

    ircomStart();

    // show selector choosen
    int i;
    long int j;
    for (i = 0; i < selector; i++)
    {
	e_led_clear();
	
	for(j = 0; j < 200000; j++)
	    asm("nop");
	
	e_set_led(i%8, 1);
	
	for(j = 0; j < 300000; j++)
	    asm("nop");

	e_led_clear();

	for(j = 0; j < 300000; j++)
	    asm("nop");
    }

    // activate obstacle avoidance
    e_activate_agenda(obstacleAvoidance, 10000);

    // acting as sender
    if (selector == 1)
    {    	
	btcomSendString("==== EMITTER ====\n\n");

	int i;
	for (i = 0; i < 10000; i++)
	{
	    // takes ~15knops for a 32window, avoid putting messages too close...
	    for(j = 0; j < 200000; j++)	asm("nop");

	    ircomWord w;
	    ircomInt2Bin(i % 256, w);
	    ircomSend(w);
	    
	    while (ircomSendDone() == 0);

	    btcomSendString(".");
	}
    }
    
    // acting as receiver
    else if (selector == 2)
    {
	btcomSendString("==== RECEIVER ====\n\n");

	int i;
	for (i = 0; i < 10000; i++)
	{
	    ircomReceive();
	    while (ircomReceiveDone() == 0);
		if (ircomReceiveData.error == 0)
		{
			btcomSendString("Receive successful : ");		
			/* Send Value*/
			int n = ircomBin2Int(ircomReceiveData.word);
			btcomSendInt(n);
		
			/* Add distance and angle */
			char tmp[64];
			sprintf(tmp," Dist: %f, Angl: %f", (double)ircomReceiveData.distance, (double)ircomReceiveData.direction);
			// sprintf(tmp,"%f", ircomReceiveData.distance);
			btcomSendString(tmp);
	
			btcomSendString("\n");		
	    }
	    else
    	{
			btcomSendString("Receive failed \n");		
	    }
    }
	}
    
    else 
    {
	int i = 0;
	long int j;
	while(1)
	{
	    e_led_clear();

	    for(j = 0; j < 200000; j++)
		asm("nop");

	    e_set_led(i, 1);

	    for(j = 0; j < 300000; j++)
		asm("nop");
	    
	    i++;
	    i = i%8;
	}	
    }    
    
    ircomStop();
    return 0;
}

int obstacleAvoidanceThreshold = 30.0;
int obstacleAvoidanceSpeed = 0.0;
//int obstacleAvoidanceSpeed = 500.0;
void obstacleAvoidance()
{    
    // check if an obstacle is perceived 
    double reading = 0.0;
    int obstaclePerceived = 0;
    int i=0;
    double x = 0.0, y = 0.0;
    for (i = 0; i < 8; i++)
    {
        reading = e_get_calibrated_prox(i);
	// if signal above noise
	if(reading >= obstacleAvoidanceThreshold)
	{
	    obstaclePerceived = 1;
	    
	    // compute direction to escape
	    double signal = reading - obstacleAvoidanceThreshold;
	    x += -cos(sensorDir[i]) * signal / 8.0;
	    y += sin(sensorDir[i]) * signal / 8.0;
	}
    }

    // no obstacles to avoid, return immediately
    if (obstaclePerceived == 0)
    {
	// go straight forward
	// change movement direction
	e_set_speed_left(obstacleAvoidanceSpeed);
	e_set_speed_right(obstacleAvoidanceSpeed);
	// return obstaclePerceived;
	return;
    }

    double desiredAngle = atan2 (y, x);
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    
    // turn left
    if (desiredAngle >= 0.0)
    {
	leftSpeed  = cos(desiredAngle);
	rightSpeed = 1;
    }
    // turn right
    else
    {
	leftSpeed = 1;
	rightSpeed = cos(desiredAngle);
    }
    
    // rescale values
    leftSpeed *= obstacleAvoidanceSpeed;
    rightSpeed *= obstacleAvoidanceSpeed;

    // change movement direction
    e_set_speed_left(leftSpeed);
    e_set_speed_right(rightSpeed);

    // advertise obstacle avoidance in progress
    // return 1;
}
