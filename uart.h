
#ifndef uart_h
#define uart_h


void uart_init(unsigned int baud);

void uart_send(char data);

void uart_send_string(const char* str);

#endif
