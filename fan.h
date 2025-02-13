#ifndef fan_h
#define fan_h

#define LIFT_FAN_PIN PD6      // lift fan connected to P4
#define THRUST_FAN_PIN PD5    // thrust fan connected to P3

void fan_init();

void lift_fan(bool on);

void thrust_fan(bool on);

void set_fan_speed(uint8_t duty_cycle);

#endif