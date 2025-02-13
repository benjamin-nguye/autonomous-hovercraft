#ifndef us_h
#define us_h

#define SIDE_TRIG PB5
#define SIDE_ECHO PD3
#define FRONT_TRIG PB3
#define FRONT_ECHO PD2

void us_init();

void trigger_sensor(int TRIG_PIN);

long pulse_length(int ECHO_PIN);

float us_getDistance(long pulse);

float us_read(bool front);

#endif