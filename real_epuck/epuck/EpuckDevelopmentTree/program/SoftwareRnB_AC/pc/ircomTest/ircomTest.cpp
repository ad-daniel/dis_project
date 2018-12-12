#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <regex.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <iostream>
using namespace std;

unsigned int crc16(char *data_p, unsigned int length);
void connect_to_epuck(int sock);
void get_bluetooth_devices(int sock);
int connect_to_epuck_MAC(char* MAC);
int write_to_epuck(int sock1, char* string, int size);
int read_from_epuck(int sock1, unsigned char* string, int size);
char* get_mac_from_number(char* number);



#define POLY 0x8408   /* 1021H bit reversed */

typedef struct devices {
	int num;
	char *name;
	char *address;
	struct devices *next;
}devices;

devices *list_dev;

unsigned int crc16(char *data_p, unsigned int length){
	unsigned char i;
	unsigned int data;
	unsigned int crc = 0xffff;
	
	if (length == 0)
		return (~crc);
	do
	{
		for (i=0, data=(unsigned int)0xff & *data_p++;
				   i < 8; 
				   i++, data >>= 1)
		{
			if ((crc & 0x0001) ^ (data & 0x0001))
				crc = (crc >> 1) ^ POLY;
			else  crc >>= 1;
		}
	} while (--length);
	
	crc = ~crc;
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xff);
	
	return (crc);
}

void connect_to_epuck(int sock){	
	struct sockaddr_rc loc_addr = { 0 };
	int num;	
	sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);	
	printf("Which one do you want to connect to ? ");
	scanf("%d",&num);	
	while(list_dev->num != num){
		if (list_dev->next != NULL) list_dev = list_dev->next;
	}
	str2ba(list_dev->address, &loc_addr.rc_bdaddr );
	loc_addr.rc_family = AF_BLUETOOTH;
	loc_addr.rc_channel = (uint8_t) 1;
	printf("Connect to the e-puck %i\n", num);
	connect(sock, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
}

void get_bluetooth_devices(int sock){
	
	printf("Searching for BT devices ... \n");
	devices* current_dev;
	inquiry_info *ii = NULL;
	int max_rsp, num_rsp;
	int dev_id, len, flags;
	int i,j;
	char addr[19] = { 0 };
	char name[248] = { 0 };
	regex_t preg;
	const char *str_regex = "e-puck_[0-9]{4}";

	dev_id = hci_get_route(NULL);
	sock = hci_open_dev( dev_id );
	if (dev_id < 0 || sock < 0) {
		perror("opening socket");
		exit(1);
	}

	len  = 8;
	max_rsp = 255;
	flags = IREQ_CACHE_FLUSH;
	ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));
	
	num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
	
	if( num_rsp < 0 ) perror("hci_inquiry");
	
	for (i = 0, j = 0; i < num_rsp; i++) {
		ba2str(&(ii+i)->bdaddr, addr);
		memset(name, 0, sizeof(name));
		if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), name, 0) < 0)
			strcpy(name, "[unknown]");
		const char *str_request = name;
		regcomp (&preg, str_regex, REG_NOSUB | REG_EXTENDED);
		if(regexec (&preg, str_request, 0, NULL, 0) == 0 ){
			if (j == 0){
				list_dev = (devices*)malloc(sizeof(devices));
				list_dev->name = (char*) malloc(20*sizeof(char));
				list_dev->address = (char*) malloc(20*sizeof(char));
				list_dev->num = j;
				strcpy(list_dev->name, name);
				strcpy(list_dev->address, addr);
				list_dev->next = NULL;
				current_dev=list_dev;
			}else{
				current_dev->next =  (devices*)malloc(sizeof(devices));
				current_dev->next->name = (char*) malloc(20*sizeof(char));
				current_dev->next->address = (char*) malloc(20*sizeof(char));
				current_dev = current_dev->next;
				current_dev->num = j;
				strcpy(current_dev->name, name);
				strcpy(current_dev->address, addr);
				current_dev->next=NULL;
			}
			printf("%i  %s  %s\n", current_dev->num, current_dev->address, current_dev->name);
			j++;
		}
	}
	free(ii);
	close(sock);
}

int connect_to_epuck_MAC(char* MAC){	
	struct sockaddr_rc loc_addr = { 0 };
	int sock1 = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	str2ba(MAC, &loc_addr.rc_bdaddr);
	loc_addr.rc_family = AF_BLUETOOTH;
	loc_addr.rc_channel = (uint8_t) 1;
	printf("Connect to the e-puck %s with socket %d\n", MAC, sock1);
	if(connect(sock1, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) == -1)
		return -1;
	else 
		return sock1;
	
}

int write_to_epuck(int sock1, char* string, int size){
	int bytes_written=0;
	bytes_written = write(sock1, string, size);
	return bytes_written;
}

int read_from_epuck(int sock1, unsigned char* string, int size){
	int bytes_read=0;
	bytes_read = read(sock1, string, size);
	return bytes_read;
}

char* get_mac_from_number(char* number){
	FILE* rfcomm;
	//int size = 1024;
	int size = 30000;
	char *text = (char *)malloc(size);
	char temp[100] = { 0 };
	int err=0, match=0;
	regex_t preg;
	strcpy(temp,"rfcomm");
	strcat(temp, number);
	strcat(temp," \\{[^\\}]*device (([0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}:[0-9A-F]{2}));");
	const char *str_regex = temp;
	rfcomm = fopen("/etc/bluetooth/rfcomm.conf","r");
	fread(text, 1, size, rfcomm);
	const char *str_request = text;

    cout << "/" << str_regex << "/" << endl;

	err = regcomp (&preg, str_regex, REG_EXTENDED & ~REG_NEWLINE);

	if (err == 0)
	{
		size_t nmatch = 1;
		regmatch_t *pmatch = NULL;
		nmatch = preg.re_nsub;
		pmatch = (regmatch_t*) malloc (sizeof (*pmatch) * nmatch);
		if (pmatch)
		{
			match = regexec (&preg, str_request, nmatch, pmatch, 0);
			regfree (&preg);

			if (match == 0)
			{
				char *macAddress = NULL;
				int start = pmatch[1].rm_so;
				int end = pmatch[1].rm_eo;
				size_t size = end - start;
				macAddress = (char*) malloc (sizeof (*macAddress) * (size + 1));
				if (macAddress)
				{
					strncpy (macAddress, &str_request[start], size);
					macAddress[size] = '\0';
				}
                cout << "rfcomm" << number << " = " << macAddress << endl;
				return macAddress;
			}else{
				printf("Error : device not found in rfcomm.conf\nType ./debug_interface to get help\n");
				return "1";
			}
		}
	}
	return "1";
}



int main(int argc, char **argv)
{
    unsigned char buf[6000] = { 0 };
    int socket[12][2] = {{0}};
    char *macAddress;
    char number[12][5] = {{0}};
    char tmp[1] = {0};
    int i;
    if(argc >= 2) {
	for(i=0 ; i<argc-1; ++i)
	    strcpy(number[i], argv[i+1]);
    }else{
	printf("Specify the number of the epuck\n");
	exit(EXIT_FAILURE);
    }
	
	
    // create the sockets for every specified e-puck
    for(i=0 ; i<argc-1; ++i)
    {
	macAddress = get_mac_from_number(number[i]);
	if (macAddress == "1") exit(EXIT_FAILURE);
		
	socket[i][0] = atoi(number[i]);
	socket[i][1] = connect_to_epuck_MAC(macAddress);
	if (socket[i][1] == -1)
	{
	    printf("Error while creating the socket for e-puck %d\n",socket[i][0]);
	    exit(EXIT_FAILURE);
	}
    }
	
    printf("Press enter to start\n");
    scanf("%c",&tmp[0]);
    for (i = 0; i < argc - 1; i++)
    {
	// send start command to e-puck
	write_to_epuck(socket[i][1], "s", 1);
    }


    // read message from e-puck, if any
    
    while(1)
    {
	for (i = 0; i < argc - 1; i++)
	{
	    int n = read_from_epuck(socket[i][1], buf, sizeof(buf));
	    buf[n] = '\0';
	    if (n > 0)
		std::cout << buf;
	}
	usleep(2500);
    }

    for (i = 0; i < argc - 1; i++)
    {
	close(socket[i][1]);
    }

    return 0;
}


