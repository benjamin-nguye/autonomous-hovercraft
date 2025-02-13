#include "ir.h"
#include <avr/io.h>
#include <stdint.h>

// LIBRARY FOR THE IR SENSOR CODE
// IR SENSOR CONNECTED TO P14
// CAN CHANGE PIN IN READ_ADC

// run this line first
void ir_adc_setup() {
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler at 64 for 250 kHz
    ADMUX |= (1 << REFS0); // Vcc as reference
}


// reads value from sensor
uint16_t read_adc() {
    ADMUX &= 0xF0; // Clear lower 4 bits to select ADC channel
    ADMUX |= (0x02); // Select ADC2 (change this line to select ADC2)
    ADCSRA |= (1 << ADSC); // Start ADC conversion
    while (!(ADCSRA & (1 << ADIF))); // Wait for conversion to complete
    ADCSRA |= (1 << ADIF); // Clear ADIF for next conversion
    return ADC; // Return ADC value
}

// maps value from sensor to distance in centimeter
uint8_t map_adc_to_distance(uint16_t adc_value) {
  if (adc_value > 415) {
    return 15;
  } else if (adc_value < 150) {
    return 70;  
  } else {
    return adc_value * (-25.0/250) + 73;
  }
}


// USE THIS FUNCTION FOR DIRECTLY READING + MAPPING
uint8_t ir_read_distance() {
  uint16_t adc = read_adc();
  uint8_t distance = map_adc_to_distance(adc);

  return distance;
}


