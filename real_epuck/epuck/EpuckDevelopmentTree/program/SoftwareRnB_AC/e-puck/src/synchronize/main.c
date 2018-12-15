
#include <e_ad_conv.h>
#include <e_init_port.h>
#include <e_epuck_ports.h>
#include <e_uart_char.h>
#include <e_led.h>

#include <e_led.h>
#include <e_motors.h>
#include <e_agenda.h>

#include <stdio.h>
#include <stdlib.h>
#include <ircom.h>
#include <btcom.h>
#include <math.h>


#define M_PI 3.141592654
#define NB_NEIGHBOURS 8

//number of motor steps to make a complete turn
#define NUMBER_STEPS_FULL_TURN 1693.32 
#define ATTRACTION_DIST 25.0

#define NEIGHBOUR_TTL 3000
#define COM_CYCLE_SPEED 600
#define TURNSPEED 100.0
#define SPEED 0.0
#define MOVE_THRESOLD 0.0
#define ORIENTATION_DELTA 5.0 * M_PI / 180.0

typedef struct
{
    int id;
    float angle;
    int sensor;
    float distance;
    long lastSeen;
} Neighbour;

Neighbour neighbours[NB_NEIGHBOURS];

float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};


int id;
//float direction = 0.0;

float direction = 0.0;
double turnspeed = TURNSPEED;
double speed = SPEED;

float attractionX;
float attractionY;
float orientationX;
float orientationY;

int nbNeighboursClose;
int nbNeighboursFar;

unsigned long lastClock;
volatile unsigned long clock;



int getRand(int a, int b);
void AddNeighbour(int id, float angle, int dir, float distance);
void processNewMessage( void );
void sendData(int msg);
void sendAngleToNeighbours(void);
void sendId(void);
void sendAngleToId(int value);
int getselector();

void move();
void perception ();
void updateDirections (float angle);


int main()
{
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();

    // activate timer
    T3CON = 0;                    // 
    T3CONbits.TCKPS = 3;          // prescsaler 
    TMR3 = 0;                     // clear timer 3
    PR3 = (1.0 * MILLISEC)/256.0; 
    IFS0bits.T3IF = 0;            // clear interrupt flag
    IEC0bits.T3IE = 1;            // set interrupt enable bit
    IPC1bits.T3IP = 2;            // timer3 priority should be > user code
    T3CONbits.TON = 1;            // start Timer3

    
    // initialise buffer 
    int k;
    for (k = 0; k < NB_NEIGHBOURS; k++)
	neighbours[k].id = -1;
    
    
    // wait for s to start
/*     btcomWaitForCommand('s'); */
    
    btcomSendString("-OK-\n");

    ircomStart();
    ircomDisableProximity();
    
    id = getselector();

    lastClock = clock;

    // activate movement
    e_activate_agenda(move, 2500);

    // advertise current direction
    e_led_clear();
    e_set_led(0,1);
    e_set_led(4,1);
    
    while(1)
    {
	// communication cycle
	ircomReceive();

	int messageReceived = 0;
	while(((clock - lastClock < COM_CYCLE_SPEED) || (ircomIsReceiving() == 1)))
	{
	    if(ircomReceiveDone() == 1)
	    {
		if (ircomReceiveData.error == 0)
		{
		    processNewMessage();		    
		    messageReceived = 1;
		}
		ircomReceive();
	    }
	}
	e_set_body_led(messageReceived);

	sendId();
	sendAngleToNeighbours();
	lastClock = clock;

	e_set_body_led(0);
    }

    ircomStop();
    return 0;
}

void sendId ()
{
    int msg;
    //MESSAGE Broadcast: msg(0:3) = 1111, msg(4:7) = ID
    msg = (id << 4) + 0xF;
    
    //Send frame
    sendData(msg);
}

void sendAngleToNeighbours ()
{
    int i;
    for(i = 0; i < NB_NEIGHBOURS; i++)
    {	
	if(neighbours[i].id >= 0)
	{
	    // received a message no more than 2 seconds ago 
	    if (clock - neighbours[i].lastSeen < NEIGHBOUR_TTL)
	    {
		sendAngleToId(i);
	    }
	}
    }
}

void sendAngleToId(int i)
{
    int msg = neighbours[i].id;

    // Compute angle between angle direction detected and goal direction
    float angle = direction - neighbours[i].angle;

    // put it in -pi:pi 
    while (angle < -M_PI) angle += 2.0 * M_PI;
    while (angle >= M_PI) angle -= 2.0 * M_PI;

    // put it in 0:2pi
    if (angle < 0.0)
	angle += 2.0 * M_PI;

    // Merge the message: msg(0:3) = neighbour if , msg(4:7) = angle in 15 slices
    msg = (msg << 4) + (int) (angle * 2.387324146); // 15.0 / M_PI / 2.0);
   
    // Send frame
    sendData(msg);
}

void sendData(int msg)
{
    ircomWord w;
    //Convert message int to binary
    ircomInt2Bin(msg, w);
    //Send binary
    ircomSend(w);
    //wait for sending done    
    while (ircomSendDone() == 0);

}

void processNewMessage ()
{	
    int n = ircomBin2Int(ircomReceiveData.word);
    int aux = n & 0xF;
    int nid = n >> 4;

    if (aux == 0xF)
    {
	if (nid != id)
	{
	    AddNeighbour(nid, ircomReceiveData.direction, ircomReceiveData.receivingSensor, ircomReceiveData.distance); 	
	}
    }
    else
    {
	if (nid == id)
	{
	    // convert angle put it in -pi:pi
	    float angle = (float) aux * 0.41887902047; // 2.0 * M_PI / 15.0
	    
	    if (angle >= M_PI)
		angle = angle - 2.0 * M_PI;
	    
	    // move to local mark
	    float ndir = ircomReceiveData.direction;
	    angle = ndir - M_PI + angle;		
	    while (angle < -M_PI) angle += 2.0 * M_PI;
	    while (angle >= M_PI) angle -= 2.0 * M_PI;
	    
	    // update current direction
	    float delta = angle - direction;
	    while (delta < -M_PI) delta += 2.0 * M_PI;
	    while (delta >= M_PI) delta -= 2.0 * M_PI;
	    
	    if (delta > 0)
		direction += ORIENTATION_DELTA;
	    else if (delta < 0)
		direction -= ORIENTATION_DELTA;
	    
	    while (direction < -M_PI) direction += 2.0 * M_PI;
	    while (direction >= M_PI) direction -= 2.0 * M_PI;
	}
    }
}

void AddNeighbour(int id, float angle, int sensor, float distance)
{
    int i;
    int oldest = -1;
    int free = -1;
    long oldestClock = clock;

    for (i = 0; i < NB_NEIGHBOURS; i++)
    {
	if(id == neighbours[i].id)
	{
	    neighbours[i].angle = angle;	    
	    neighbours[i].sensor = sensor;
	    neighbours[i].distance = distance;	    
	    neighbours[i].lastSeen = clock;
	    return;
	}
	else if(neighbours[i].id == -1)
	{
	    free = i;
	}
	else if(neighbours[i].lastSeen < oldestClock)
	{
	    oldestClock = neighbours[i].lastSeen;
	    oldest = i;
	}
    }

    if (free >= 0) 
	i = free;
    else
	i = oldest;

    neighbours[i].id = id;
    neighbours[i].angle = angle;
    neighbours[i].sensor = sensor;
    neighbours[i].distance = distance;	    
    neighbours[i].lastSeen = clock;
}

int getRand(int a, int b)
{
    int r;
    r = a + (rand() % (b-a));
    return r;
}


void perception ()
{
    attractionX = 0.0;
    attractionY = 0.0;
    nbNeighboursClose = 0;
    nbNeighboursFar = 0;

    // go through table of neighbours
    int i;
    for (i = 0; i < NB_NEIGHBOURS; i++)
    {
	// entry allocated ?
	if (neighbours[i].id >= 0)
	{
	    // received a message no more than 2 seconds ago 
	    if (clock - neighbours[i].lastSeen < NEIGHBOUR_TTL)
	    {
		// the neighbour is far away and we should drive towards him
		if (neighbours[i].distance >= ATTRACTION_DIST)
		{
		    // add contribution of a neighbour : 
		    attractionX += cos(neighbours[i].angle);
		    attractionY += sin(neighbours[i].angle);		    
		    nbNeighboursFar++;
		}
		// take care of orientation
		else
		{
		    // orientation vector is updated during process of messages received
		    nbNeighboursClose++;
		}
	    }
	}
    }
}

void move()
{
    perception();

    // compute final direction and motor speed  
    speed = 0.0;   
    float desiredAngle = direction;
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    // if angle is not big ... don't turn
    if (fabs(desiredAngle) < MOVE_THRESOLD * M_PI / 180.0)
    {
	// direction does not change...
	leftSpeed = speed;
	rightSpeed = speed;
    }    
    // turn left
    else if (desiredAngle >= 0.0)
    {
	leftSpeed = -turnspeed;
	rightSpeed = turnspeed;

	// update internal direction according to movement
	float timestep = 0.25;
	float angle = 2.0 * M_PI * turnspeed * timestep / NUMBER_STEPS_FULL_TURN;
	updateDirections(-angle);
    }
    // turn right
    else
    {
	leftSpeed = turnspeed;
	rightSpeed = -turnspeed;

	// update internal direction according to movement
	float timestep = 0.25;
	float angle = 2.0 * M_PI * turnspeed * timestep / NUMBER_STEPS_FULL_TURN;
	updateDirections(angle);
    }
    
    // change movement direction
    e_set_speed_left(leftSpeed);
    e_set_speed_right(rightSpeed);
}

void updateDirections (float angle)
{
    // update own's global desired direction according to movement
    direction += angle;
    while (direction < -M_PI) direction += 2.0 * M_PI;
    while (direction >= M_PI) direction -= 2.0 * M_PI;

    // update also estimated direction of friends (or neighbours)
    int  i;
    for (i = 0; i < NB_NEIGHBOURS; i++)
    {
	if (neighbours[i].id >= 0)
	{
	    float a = neighbours[i].angle;
	    
	    a += angle;
	    while (a < -M_PI) a += 2.0 * M_PI;
	    while (a >= M_PI) a -= 2.0 * M_PI;

	    int s = (int)((a + M_PI) / (2.0 * M_PI / 8.0));

	    neighbours[i].angle = a;
	    neighbours[i].sensor = s;
	}
    }
}

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void _ISRFAST _T3Interrupt(void)
{
    // clear interrupt flag
    IFS0bits.T3IF = 0;
    
    clock++;
}
