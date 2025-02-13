#ifndef ir_h
#define ir_h

#include <stdint.h>

void ir_adc_setup();

uint16_t read_adc();

uint8_t map_adc_to_distance(uint16_t adc_value);

uint8_t ir_read_distance();

#endif