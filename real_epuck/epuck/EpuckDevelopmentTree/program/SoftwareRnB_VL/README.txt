




####################################
##  CHANGES FROM LIBRARY VERSION  ##
####################################


A_D
==========

e_ad_conv.c:
e_ad_conv.h:
    buffered scanning version


UART
==========

e_epuck_ports.inc: 
    unchanged

e_init_uart1.s:
e_init_uart2.s:
    added reset to zero of two variable counters not in library version.
        .extern _U1RXRcvCnt 
        .extern _U1RXReadCnt

e_uart1_rx_char.s:
e_uart2_rx_char.s:
    significant changes

e_uart1_tx_char.s:
    section marked near
e_uart2_tx_char.s:
    section marked near
    interrupt changed (IEC0->IEC1)

e_uart_char.h:
    added ischar functions
    added macros send_static_text & send_text
    added int_clr_addr & int_clr_mask
