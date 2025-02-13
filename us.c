#include <avr/io.h>
#include <stdbool.h>
#include "us.h"



void us_init(){
	//front USS
	DDRB |= (1 << FRONT_TRIG); // Set TRIG_PIN as output
	DDRD &= ~(1 << FRONT_ECHO); // Set ECHO_PIN as input

  //side USS
  DDRB |= (1 << SIDE_TRIG); // Set TRIG_PIN as output
  DDRD &= ~(1 << SIDE_ECHO); // Set ECHO_PIN as input 
}

void trigger_sensor(int TRIG_PIN){ //TRIG_PIN is a variable in this case to use the same function twice
	PORTB &= ~(1 << TRIG_PIN);
	_delay_us(2);
 
	// Send a 10µs pulse to the trigger pin
  PORTB |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTB &= ~(1 << TRIG_PIN);
}


long pulse_length(int ECHO_PIN) { //ECHO_PIN is a variable in this case to use the same function twice

	long duration = 0;

	while (!(PIND & (1 << ECHO_PIN)));

	while (PIND & (1 << ECHO_PIN)) {
		duration++;
		_delay_us(0.05);

    if(duration == 40000){ //overflow at  686 cm idk you can change this, stops unnecessary waiting
      return duration;
    }
	}
	
	return duration;
}

float us_getDistance(long pulse){
  return ((pulse * 0.0343) / 2.0);
}

// call this ****
// if front = 1: reads front sensor
// if front = 0: reads side
float us_read(bool front) {

  long pulse;
  if (front) {
    trigger_sensor(FRONT_TRIG); //sends pulse to the trigger
    pulse = pulse_length(FRONT_ECHO); //see how long the pulse takes to get to the echo
  } else {
    trigger_sensor(SIDE_TRIG); //sends pulse to the trigger
    pulse = pulse_length(SIDE_ECHO); //see how long the pulse takes to get to the echo
  }
  float distance = us_getDistance(pulse);
  return distance;
  
}
