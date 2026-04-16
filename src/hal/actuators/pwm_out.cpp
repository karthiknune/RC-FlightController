#include "hal/actuators/pwm_out.h"
#include "config.h"

// Sets the ESC as percentage. 
// -1 will send a low signal to initialize the ESC.
// Note: duty values scaled from MicroPython 0-1023 range to 10-bit C range.
void setThrottle(int pulse) {
  if(pulse < 0) {
    pulse = THROTTLE_INIT;
  } else {
    pulse = constrain(pulse,THROTTLE_MIN,THROTTLE_MAX);
  }
  ledcWrite(esc_channel, US_TO_DUTY(pulse));
}

void setAileron(int pulse) {
}

// Sets angle of rudder between -40 and 40 deg.
void setRudder(int pulse) {
  pulse = constrain(pulse,RUDDER_MIN,RUDDER_MAX);
  ledcWrite(rudder_channel, US_TO_DUTY(pulse));
}

// Sets angle of elevator between -10 and 15 deg.
void setElevator(int pulse) {
  pulse = constrain(pulse,ELEVATOR_MIN,ELEVATOR_MAX);
  ledcWrite(elevator_channel, US_TO_DUTY(pulse));
}

// Convert PWM to Airfoil Deflection Angle
float pwmToThrottle(unsigned long pw) {
  if(pw < SERVO_MIN) {
    return NAN;
  }
  return (((long)pw - THROTTLE_INT) / THROTTLE_SLOPE);
}

// Convert PWM to Airfoil Deflection Angle
float pwmToRudder(unsigned long pw) {
  if(pw < SERVO_MIN) {
    return NAN;
  }
  return (((long)pw - RUDDER_INT) / RUDDER_SLOPE);
}

// Convert PWM to Airfoil Deflection Angle
float pwmToElevator(unsigned long pw) {
  if(pw < SERVO_MIN) {
    return NAN;
  }
  return (((long)pw - ELEVATOR_INT) / ELEVATOR_SLOPE);
}

void pwm_init() {
    // Set up PWM channels with a frequency of 50Hz and 16-bit resolution
    ledcSetup(esc_channel, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(aileron_channel, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(elevator_channel, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(rudder_channel, PWM_FREQ, PWM_RESOLUTION);

    //////attach the channels to the corresponding GPIO pins
    ledcAttachPin(esc_pin, esc_channel);
    ledcAttachPin(aileron_pin, aileron_channel);
    ledcAttachPin(elevator_pin, elevator_channel);
    ledcAttachPin(rudder_pin, rudder_channel);

    pwm_reset();
}

void pwm_reset() {
    //  neutral positions
    setThrottle(-1);
    setElevator(0);
    setRudder(0);
}
