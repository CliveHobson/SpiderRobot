// Locate the initial position of legs
// RegisHsu 2015-09-09
//
// Use PWM module to initialise servo positions instead of discrete pins.
//   Use Adafruit_PWMServoDriver module and PWM hardware (PCA9685) to control
//   servos instead of Servo module and discrete pins.
//   CliveHobson 2018-07-11

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

/* Servos --------------------------------------------------------------------*/
#define SERVO_VIA_PWM 1
// define 12 servos for 4 legs
#if SERVO_VIA_PWM
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const float PWM_FREQUENCY = 50;
const uint16_t PWM_SERVO_MIN = 103;
const uint16_t PWM_SERVO_MAX = 512;
const int servo_address[4][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14} };
#else
Servo servo[4][3];
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };
#endif

void servo_attach(void)
{
#if SERVO_VIA_PWM
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);
  Wire.setClock(400000);
#else
  for (int leg = 0; leg < 4; ++leg)
  {
    for (int joint = 0; joint < 3; ++joint)
    {
      servo[leg][joint].attach(servo_pin[leg][joint]);
      delay(100);
    }
  }
#endif
}

/*
  - convert polar (degrees) to PWM "on" time
   ---------------------------------------------------------------------------*/
uint16_t polar_to_pwm(float angle)
{
  // multiply by 10 to get better precision
  return constrain(map(static_cast<uint16_t>(angle * 10.0), 0, 1800, PWM_SERVO_MIN, PWM_SERVO_MAX),
                   PWM_SERVO_MIN,
                   PWM_SERVO_MAX);
}

/*
  - send the joint movement instruction to hardware
   ---------------------------------------------------------------------------*/
void drive_servo(int leg, int joint, float angle)
{
#if SERVO_VIA_PWM
  pwm.setPWM(servo_address[leg][joint], 0, polar_to_pwm(angle));
#else
  servo[leg][joint].write(angle);
#endif
}

/*
  - drive all servos to 90 degrees (central position)
   ---------------------------------------------------------------------------*/
void drive_all_90()
{
  for (int leg = 0; leg < 4; ++leg)
  {
    for (int joint = 0; joint < 3; ++joint)
    {
      drive_servo(leg, joint, 90);
      delay(20);
    }
  }
}

void setup()
{
  servo_attach();
  drive_all_90();
}

void loop(void)
{
  delay(100);
}
