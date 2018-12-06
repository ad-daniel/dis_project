/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  AROB2008.c
 *
 *  Controller for using e-puck as an end effector for Vlad Trifa's webservices.
 *
 *  2007-10-20 cianci
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <p30f6014a.h>

//#define FLOOR_SENSORS	// define to enable floor sensors

#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>

#include <uart/e_uart_char.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_micro.h>
#include <motor_led/advance_one_timer/e_agenda.h>
#include <camera/fast_2_timer/e_po3030k.h>
#include <codec/e_sound.h>

#include <contrib/radio_swis/e_radio_swis.h>
#include <contrib/robot_id/e_robot_id.h>

#ifdef FLOOR_SENSORS
#include <./I2C/e_I2C_protocol.h>
#endif 

#define uart_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)


//static char buffer[52*39*2+3+80];
	
extern int e_mic_scan[3][MIC_SAMP_NB];
extern unsigned int e_last_mic_scan_id;



void calibrate_ir_sensors(int ir_delta[8])
{
	int i=0,j=0;
	int long t;
	int long tmp[8];
	e_set_led(8,1);
	for (t=0;t<2000000;++t);
	e_led_clear();
	for (t=0;t<200000;++t);


	for (;i<8;++i) {
		ir_delta[i]=0;
		tmp[i]=0;
	}

	for (;j<100;++j) {
		for (i=0;i<8;++i) {
			tmp[i]+=(e_get_prox(i));
			for (t=0;t<1000;++t);
		}
	}

	for (i=0;i<8;++i) {
		ir_delta[i]=(int)(tmp[i]/(j*1.0));
	}
	
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct _WebSvcInitMsg {
    unsigned char Source;
    unsigned char DeviceType;
} WebSvcInitMsg;
#define AM_WEBSVCINITMSG 100

void init(void) 
{
	e_init_port();    // configure port pins
	e_start_agendas_processing();
	e_init_motors();
	e_init_uart1();   // initialize UART to 115200 Kbaud
	e_init_ad_scan();
	e_init_robot_id();
    
    #ifdef FLOOR_SENSORS
	e_I2Cp_init();
    #endif

	if(RCONbits.POR) {	// reset if power on (some problem for few robots)
		RCONbits.POR=0;
		RESET();
	}

    e_acc_calibr();

    // start the radio module.
    e_radio_swis_init(RADIO_SWIS_DEFAULT_GROUP, e_get_robot_id(),
            RADIO_SWIS_HW_ATTENUATOR_0DB, RADIO_SWIS_MAX_TXPWR);

    // build "I'm here" announcement message, and send it on the radio.
    WebSvcInitMsg msg;
    msg.Source = e_get_robot_id();
    msg.DeviceType = 1;
    e_radio_swis_send_msgtype(e_radio_swis_get_group(), RADIO_SWIS_BASESTATION,
            (unsigned char*)&msg, sizeof(WebSvcInitMsg), AM_WEBSVCINITMSG);

    // keep sending it until we receive an ack from the basestation.
    unsigned char pkt[RADIO_SWIS_MAXSIZE];
    unsigned int size,i;
    while ( e_radio_swis_packet_ready(pkt,&size) == 0 ) {
        for(i=0; i<65535; i++) Nop(); // wait loop!
        e_radio_swis_send_msgtype(e_radio_swis_get_group(), 
                RADIO_SWIS_BASESTATION, (unsigned char*)&msg, 
                sizeof(WebSvcInitMsg), AM_WEBSVCINITMSG);
    }

    // TODO: check to make sure pkt really was an ack???

}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct _WebSvcCmdMsg {
    unsigned char Source;
    unsigned char Cmd[107];
} WebSvcCmdMsg;
#define AM_WEBSVCCMDMSG 102

typedef struct _WebSvcDataMsg {
    unsigned char Source;
    unsigned char Data[107];
} WebSvcDataMsg;
#define AM_WEBSVCDATAMSG 103

int main(void)
{
	//char c,c1,c2;
	//int	i,j,n,
    int speedr,speedl,positionr,positionl,
        LED_action,accx,accy,accz,selector,sound;
    //LED_nbr,
	static char first=0;
	int ir_delta[8]={0,0,0,0,0,0,0,0};  // sensor offsets
    /*
	char *address;
	char *ptr;
	TypeAccSpheric accelero;
    */

    unsigned char pkt[RADIO_SWIS_MAXSIZE];
    int size;

    init(); // do all the setup stuff

	
	while(1) { // then loop for ever (waiting for and responding to pkts)

            // prepare packet for formulating response
            WebSvcDataMsg msg;
            msg.Source = e_get_robot_id();
            // msg.Data is filled in the switch below
            
        if(e_radio_swis_packet_ready(pkt,&size) && size==sizeof(WebSvcCmdMsg)) {
            //assert((WebSvcCmdMsg)pkt->Source == 0);
            char *cmd = ((WebSvcCmdMsg*)pkt)->Cmd;

			cmd[0]=toupper(cmd[0]); // we also accept lowercase letters
			switch (cmd[0]) {
			case 'A': // read accelerometer
				accx=e_get_acc(0);
				accy=e_get_acc(1);
				accz=e_get_acc(2);
				sprintf(msg.Data,"a,%d,%d,%d",accx,accy,accz);				
				break;
			case 'B': // set body led
				sscanf(cmd,"B,%d",&LED_action);
			 	e_set_body_led(LED_action);
				sprintf(msg.Data,"b");				
				break;
			case 'C': // read selector position
				selector = SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
				sprintf(msg.Data,"c,%d",selector);
				break;
			case 'D': // set motor speed
				sscanf(cmd, "D,%d,%d", &speedl, &speedr);
				e_set_speed_left(speedl);
				e_set_speed_right(speedr);
				sprintf(msg.Data,"d");				
				break;
			case 'E': // read motor speed
				sprintf(msg.Data,"e,%d,%d",speedl,speedr);
				break; 
			case 'F': // set front led
				sscanf(cmd,"F,%d",&LED_action);
				e_set_front_led(LED_action);
				sprintf(msg.Data,"f");				
				break;
			case 'K':  // calibrate proximity sensors
				//uart_send_static_text("k, Starting calibration - Remove any object in sensors range\r\n");
				calibrate_ir_sensors(ir_delta);
				sprintf(msg.Data,"k");				
				//uart_send_static_text("k, Calibration finished\r\n");
				break;
			case 'L': // set led
                {
                int ledarray[8], i;
				sscanf(cmd,"L,%d,%d,%d,%d,%d,%d,%d,%d",&ledarray[0],&ledarray[1],&ledarray[2],&ledarray[3],&ledarray[4],&ledarray[5],&ledarray[6],&ledarray[7]);
                for (i=0; i<8; i++) {
                    e_set_led(i,ledarray[i]);
                    ledarray[i] = e_get_led(i);
                }
				sprintf(msg.Data,"L,%d,%d,%d,%d,%d,%d,%d,%d",ledarray[0],ledarray[1],ledarray[2],ledarray[3],ledarray[4],ledarray[5],ledarray[6],ledarray[7]);
                }
				break;
                /*
			case 'M': // read floor sensors (optional)
#ifdef FLOOR_SENSORS
				e_I2Cp_enable();
				for (i=0; i<6; i++)	buffer[i] = e_I2Cp_read(0xC0,i);
				e_I2Cp_disable();
				sprintf(buffer,"m,%d,%d,%d\r\n",
				(unsigned int)buffer[1] | ((unsigned int)buffer[0] << 8),
				(unsigned int)buffer[3] | ((unsigned int)buffer[2] << 8),
				(unsigned int)buffer[5] | ((unsigned int)buffer[4] << 8));
				uart_send_text(buffer);
#else
				uart_send_static_text("m,0,0,0\r\n");
#endif
				break;
                */
			case 'N': // read proximity sensors
				sprintf(msg.Data,"n,%d,%d,%d,%d,%d,%d,%d,%d",
				        e_get_prox(0)-ir_delta[0],e_get_prox(1)-ir_delta[1],e_get_prox(2)-ir_delta[2],e_get_prox(3)-ir_delta[3],
				        e_get_prox(4)-ir_delta[4],e_get_prox(5)-ir_delta[5],e_get_prox(6)-ir_delta[6],e_get_prox(7)-ir_delta[7]);
				break;
			case 'O': // read ambient light sensors
				sprintf(msg.Data,"o,%d,%d,%d,%d,%d,%d,%d,%d",
				        e_get_ambient_light(0),e_get_ambient_light(1),e_get_ambient_light(2),e_get_ambient_light(3),
				        e_get_ambient_light(4),e_get_ambient_light(5),e_get_ambient_light(6),e_get_ambient_light(7));
				break;
			case 'P': // set motor position
				sscanf(cmd,"P,%d,%d",&positionl,&positionr);
				e_set_steps_left(positionl);
				e_set_steps_right(positionr);
				sprintf(msg.Data,"p");				
				break;
			case 'Q': // read motor position
				sprintf(msg.Data,"q,%d,%d",e_get_steps_left(),e_get_steps_right());
				break;
			case 'R': // reset
				sprintf(msg.Data,"r");				
				RESET();
				break;
			case 'S': // stop
				e_set_speed_left(0);
				e_set_speed_right(0);
				e_set_led(8,0);
				
				sprintf(msg.Data,"s");				
				break;
			case 'T': // stop
				sscanf(cmd,"T,%d",&sound);
				if(first==0){
					e_init_sound();
					first=1;
				}
				switch(sound)
				{
					case 1: e_play_sound(0,2112);break;
					case 2: e_play_sound(2116,1760);break;
					case 3: e_play_sound(3878,3412);break;
					case 4: e_play_sound(7294,3732);break;
					case 5: e_play_sound(11028,8016);break;
					default:
						e_close_sound();
						first=0;
						break;
				}		
				sprintf(msg.Data,"t");				
				break;
			case 'U':
				sprintf(msg.Data,"u,%d,%d,%d",e_get_micro_volume(0),e_get_micro_volume(1),e_get_micro_volume(2));
				break;
			case 'V': // get version information
				sprintf(msg.Data,"v,Version 1.1.3 September 2006");
				break;
            case 'X': // WEBSVC BEHAVOIR!!!
                break;
            // Y = new robot!
			default:
				sprintf(msg.Data,"z,Command not found");				
				break;
			}

            // send response packet
            e_radio_swis_send_msgtype(e_radio_swis_get_group(), 
                RADIO_SWIS_BASESTATION, (unsigned char*)&msg, 
                sizeof(WebSvcDataMsg), AM_WEBSVCDATAMSG);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

        } // if packet received

	} // while
} // main
