#include <avr/io.h>
#include <stdbool.h>
#include "fan.h"

// LIBRARY FOR FANS

// run this first
// set fans as output
void fan_init() {
  DDRD |= (1 << LIFT_FAN_PIN);
  DDRD |= (1 << THRUST_FAN_PIN);

  // set pwm for thrust fan
  
  TCCR0A |= (1 << COM0A1) | (1<<COM0B1) | (1 << WGM01) | (1 << WGM00); // Non-inverting mode, Fast PWM
  TCCR0B |= (1 << CS01); // Prescaler = 8
  

  // Set OC2A (PD3) as output
  //DDRD |= (1 << PD5);
}


// turns lift fan on or off
// if param is 1 --> turns on
// if param is 0 --> turns off
void lift_fan(bool on) {
  if (on) {
   OCR0A=255; //PORTD |= (1 << LIFT_FAN_PIN);
  } else {
    OCR0A=0;//PORTD &= ~(1 << LIFT_FAN_PIN);
  }
}

// turns thrust fan on or off
// if param is 1 --> turns on
// if param is 0 --> turns off
void thrust_fan(bool on) {
  if (on) {
   OCR0B=255;// PORTD |= (1 << THRUST_FAN_PIN);
  } else {
   OCR0B=0; //PORTD &= ~(1 << THRUST_FAN_PIN);
  }
}

// max = 255
// half = 128
void set_fan_speed_thrust(uint8_t duty_cycle){
  OCR0B = duty_cycle;
}

void set_fan_speed_lift(uint8_t duty_cycle){
  OCR0A = duty_cycle;
}