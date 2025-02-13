
//  ENGR 290  -   TEAM 18
//
//  This is the main program that runs our hovercraft.
//  It is written entirely in C with no use of any Arduino libraries
//
//  Features Include:
//
//    - Self-correcting algorithm to keep its orientation
//    - IMU gyroscope calibration
//    - Detection of path of furthest distance to ensure turning towards the right direction
//    - Finish line detection
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////


// --------------------------------------------------
// |                   INCLUDES                     |
// --------------------------------------------------

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#include "fan.c"
#include "ir.c"
#include "twi.c"
#include "uart.c"
#include "init_290.c"
#include "us.c"

#define FRONT_WALL_DISTANCE 35
#define SIDE_WALL_DISTANCE 35
#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0

// addresses for MPU6050, most of them unused
#define MPU6050_ADDR 0x68       // twi address of mpu6050
#define MPU6050_PWR_MGMT_1 0x6b 

#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48


float YawCalibrationRate;
float RateYaw;
float Yaw;
float desired_yaw = 0;

// acceleration data unused
float x_accel;
float y_accel;
float z_accel;

float front_distance;
float top_distance;

uint8_t top_scans = 0;

float left_scan;
float front_scan;
float right_scan;

// --------------------------------------------
// |           FUNCTION DEFINITIONS           |
// --------------------------------------------

void MPU6050_init(void) {

 // start and write address of MPU6050
 TWI_status = TWI_start(MPU6050_ADDR, TW_WRITE);
 if (TWI_status) return;       // checks for errors

 // send register address for wake up command (mgu6050 is asleep)
 TWI_status = TWI_write(MPU6050_PWR_MGMT_1);
 if (TWI_status) return;       // check errors

 // send the wakeup command at the given register above
 TWI_status = TWI_write(0x00);
 if (TWI_status) return;

 TWI_stop();

}

void MPU6050_read(int16_t* accel, int16_t* gyro) {

  uint8_t data[14];     // 14-byte buffer for data (6 bytes for x-y-z accel, 6 bytes for x-y-z gyro, 2 for temp)

  // use Read_Reg_N function to iterate over registers and read from them
  // because the data from the imu is in consecutive registers
  TWI_status = Read_Reg_N((uint8_t) MPU6050_ADDR, (uint8_t) MPU6050_ACCEL_XOUT_H, 14, (int16_t*) data);
  if (TWI_status) return; 

  // the data is separated in 2 separate bytes: hi and low
  // must recombine them into one 2-byte number
  accel[0] = (data[0] << 8) | data[1];
  accel[1] = (data[2] << 8) | data[3];
  accel[2] = (data[4] << 8) | data[5];

  // temperature uses 6 and 7
  gyro[0] = (data[8] << 8) | data[9];
  gyro[1] = (data[10] << 8) | data[11];
  gyro[2] = (data[12] << 8) | data[13];

  // just angular rate on z-axis
  RateYaw = (float) gyro[2] / GYRO_SCALE;

  // acceleration measurements
  // NOT USED
  x_accel = (float) accel[0] / ACCEL_SCALE - 0.02;
  y_accel = (float) accel[1] / ACCEL_SCALE;
  z_accel = (float) accel[2] / ACCEL_SCALE + 0.05;
}

void set_servo(int yaw) {
  OCR1A = Servo_angle[yaw];
}

// turns to angle mapped to Servo_angle
void turn_to(float yaw) {
  
  yaw *= 1.1;
  if (yaw < -90) yaw = -90;
  if (yaw > 90) yaw = 90;

  uint8_t index = (uint8_t)((yaw + 90.0) * 255.0 / 180.0);
  
  OCR1A = Servo_angle[index];
}


// ----------------------------------
// |              MAIN              |
// ----------------------------------

int main(void) {

  fan_init();
  gpio_init();
  timer1_50Hz_init(0);
  ir_adc_setup();
  us_init();
  twi_init();
  MPU6050_init();


  int16_t accel[3], gyro[3];
  float yaw, prev_yaw = 0;
  float yaw_error;

  int i;

  set_servo(127);

  // --------------- Calibrate Gyroscope ---------------------
  
  // this short calibration sequence for the IMU's gyroscope is property of Carbon Aeronautics 
  // (https://github.com/CarbonAeronautics/Part-V-GyroscopeCalibration/blob/main/ArduinoCode)
  for (int RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    MPU6050_read(accel, gyro);
    YawCalibrationRate += RateYaw;
    _delay_ms(1);
  }
  YawCalibrationRate /= 2000;
  

  // --------------------- Lift Off -----------------------
  
  lift_fan(1);
  set_fan_speed_thrust(96);
  _delay_ms(2000);
  
  
  // ------------------ Movement Algorithm ----------------

  bool go = 1;
  while (go) {

    // read gyro
    MPU6050_read(accel, gyro);
    // calculate yaw value using Euler method
    RateYaw *= 1.15;
    RateYaw -= YawCalibrationRate;
    yaw = prev_yaw + RateYaw * 0.1;
    prev_yaw = yaw;
    
    //read distances
    front_distance = us_read(1);
    top_distance = us_read(0);

    // correct yaw of hovercraft by turning servo to the desired yaw
    yaw_error = yaw - desired_yaw;
    if (abs(yaw_error) > 2.5) {
      turn_to(yaw_error);
    }

    // finish line detected
    if (top_distance < 25) {
      top_scans += 1;

      // three consecutive reads of under 25 cm for the top us sensor are required to stop the hovercraft
      // this avoids accidental reads which can occure from time to time
      if (top_scans == 3) go = 0;
    }
    
    
    // front wall detected
    else if (front_distance < FRONT_WALL_DISTANCE) {
      // reset consecutive top scan counter back to 0
      top_scans = 0;

      // stop hovercraft
      lift_fan(0);
      thrust_fan(0);
      _delay_ms(800);

      // scan left
      turn_to(-90);
      _delay_ms(800);
      left_scan = us_read(1);
      _delay_ms(10);

      // scan in front
      turn_to(0);
      _delay_ms(400);
      front_scan = us_read(1);
      _delay_ms(10);


      // scan right
      turn_to(90);
      _delay_ms(300);
      right_scan = us_read(1);
      _delay_ms(10);

      // reset yaw to 0
      prev_yaw = 0.;
      yaw = 0.;


      // choose yaw to follow based on direction of furthest available path
      if ((right_scan > left_scan) && (right_scan > front_scan)) {
        desired_yaw = -90.;
      } else if ((left_scan > right_scan) && (left_scan > front_scan)) {
        desired_yaw = 90.;
      } else {
        desired_yaw = 0.;
      }


      yaw_error = yaw - desired_yaw; 
      turn_to(-desired_yaw);
      // lift off to turn

      lift_fan(1);
      set_fan_speed_thrust(72);
      
      // turns until it reaches the desired new yaw
      for (int f = 0; f < 60; f++) {
        MPU6050_read(accel, gyro);
        RateYaw *= 1.15;
        RateYaw -= YawCalibrationRate;
        yaw = prev_yaw + RateYaw * 0.1;
        prev_yaw = yaw;

        yaw_error = yaw - desired_yaw;

        // reaches desired yaw
        if (abs(yaw_error) < 15.) {
          break;
        }
        if (abs(yaw_error) > 2.5) {
          turn_to(yaw_error);
        }

        _delay_ms(100);
      }

      // stops when reached desired yaw to stop momentum
      lift_fan(0);
      thrust_fan(0);
      _delay_ms(800);

      lift_fan(1);
      set_fan_speed_thrust(96);

      // go straight for a bit before restarting movement sequence
      for (i = 0; i < 20; i++) {
        MPU6050_read(accel, gyro);
        RateYaw *= 1.15;
        RateYaw -= YawCalibrationRate;
        yaw = prev_yaw + RateYaw * 0.1;
        prev_yaw = yaw;

        yaw_error = yaw - desired_yaw;
        if (abs(yaw_error) > 2.5) {
          turn_to(yaw_error);
        }

        _delay_ms(100);
      }

      // end of turning sequence
    }  

    // keep going
    else {
      top_scans = 0;
      _delay_ms(100);
    }
   
  }


  // EXITED

  lift_fan(0);
  thrust_fan(0);


  // ------------------ Infinite loop ---------------------
  while(1) {
    ;;
  }
  
}

