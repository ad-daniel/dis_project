#ifndef _UART_TXRX_CHAR
#define _UART_TXRX_CHAR

#define COM_BUFFER_LEN 80

#define uart_send_static_text(msg) { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); }
#define uart_send_text(msg) { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); }

// Init uart 1 at 115200bps, 8 data bits, 1 stop bit, Enable ISR for RX and TX
void e_init_uart1(void);

// Return the number of characters available, 0 if none are available
int  e_ischar_uart1();

// If available, read 1 char, put it in pointer and return 1. Return 0 if no char is available
int  e_getchar_uart1(char *);

// Send a buffer of char of size length
void e_send_uart1_char(const char * buff, int length);

// Return 1 if buffer sending is in progress, return 0 if not
int  e_uart1_sending(void);


// Init uart 2 at 115200bps, 8 data bits, 1 stop bit, Enable ISR for RX and TX
void e_init_uart2(void);

// Return the number of characters available, 0 if none are available
int  e_ischar_uart2();

// If available, read 1 char, put it in pointer and return 1. Return 0 if no char is available
int  e_getchar_uart2(char *);

// Send a buffer of char of size length
void e_send_uart2_char(const char * buff, int length);

// Return 1 if buffer sending is in progress, return 0 if not
int  e_uart2_sending(void);


extern void *e_uart1_int_clr_addr; //address to be clear on interrupt
extern int e_uart1_int_clr_mask; //mask to be use to clear on interrupt
extern void *e_uart2_int_clr_addr; //address to be clear on interrupt
extern int e_uart2_int_clr_mask; //mask to be use to clear on interrupt


#endif
