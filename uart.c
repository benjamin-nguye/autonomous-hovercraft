#include "uart.h"
#include <avr/io.h>


void uart_init(unsigned int baud) {
    unsigned int ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}


void uart_send(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until the buffer is empty
    UDR0 = data; // Send data
}

void uart_send_string(const char* str) {
    while (*str) {
        uart_send(*str++);
    }
}


