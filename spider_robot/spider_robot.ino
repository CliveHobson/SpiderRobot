/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use, you must to adjust the robot, in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

/* -----------------------------------------------------------------------------
  Add remote control by HC-06 bluetooth module
    Regis for spider project, 2015-09-26

  Use SerialCommands instead of SerialCommand.
    Allows easy selection of serial port for bluetooth module.
    SerialCommands is available from Arduino library.
    Ebony 2018-07-05

  Use PWM module to control servos instead of discrete pins.
    Use Adafruit_PWMServoDriver module and PWM hardware (PCA9685) to control
    servos instead of Servo module and discrete pins.
    CliveHobson 2018-07-02
   ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>    //to define and control servos
#include <FlexiTimer2.h>//to set a timer to manage all servos

/* remote control ------------------------------------------------------------*/
#include <SerialCommands.h>

/* Serial Commands------------------------------------------------------------*/
Stream & CmdSerial = Serial1;
char serial_command_buffer_[32];
SerialCommands SCmd(&Serial1, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

/* Servos --------------------------------------------------------------------*/
#define SERVO_VIA_PWM 1
//define 12 servos for 4 legs
#if SERVO_VIA_PWM
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const float PWM_FREQUENCY = 50;
const uint16_t PWM_SERVO_MIN = 103;
const uint16_t PWM_SERVO_MAX = 512;
const int servo_address[4][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14} };
#else
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };
#endif

/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;

/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;

/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
float spot_turn_speed = 4;
float leg_move_speed = 8;
float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;

/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

/* Constants for obstacle detection with Infrared or Ultrasonic --------------*/
// Select detection device
enum class ObstacleDetectorKind : int
{
  None,
  Infrared,
  Ultrasonic
};
constexpr ObstacleDetectorKind OBSTACLE_DETECT_DEVICE = ObstacleDetectorKind::Ultrasonic;
// Infrared I/O pin
const int IR_Detect_IO = 14;
// Ultrasonic Trigger / Echo pins
const int Sonic_Detect_Trig = 14;
const int Sonic_Detect_Echo = 15;
// Ultrasonic distance in Centimetres
const int Sonic_Detect_Range = 15;

/*
  - callback for SerialCommand commands for Bluetooth module
     w 1 x: forward x step
     w 2 x: back x step
     w 3 x: right turn x step
     w 4 x: left turn x step
     w 5 x: hand shake x times
     w 6 x: hand wave x times
     w 8 x: run test sequence (x is ignored for now)
     w 9 1: stand
     w 9 0: sit
   ---------------------------------------------------------------------------*/
enum class CommandAction : int
{
  Forward = 1,
  Backward = 2,
  Left = 3,
  Right = 4,
  Shake = 5,
  Wave = 6,
  Test = 8,
  StandSit = 9
};
void cmd_action(SerialCommands* sender)
{
  char *arg;
  arg = sender->Next();
  int action_mode = atoi(arg); // cannot report errors (gives zero on non-numeric text)
  arg = sender->Next();
  int n_step = atoi(arg); // cannot report errors (gives zero on non-numeric text)

  sender->GetSerial()->println("Action:");

  switch (static_cast<CommandAction>(action_mode))
  {
    case CommandAction::Forward:
      sender->GetSerial()->println("Step forward");
      if (!is_stand())
        stand();
      step_forward(n_step);
      break;
    case CommandAction::Backward:
      sender->GetSerial()->println("Step back");
      if (!is_stand())
        stand();
      step_back(n_step);
      break;
    case CommandAction::Left:
      sender->GetSerial()->println("Turn left");
      if (!is_stand())
        stand();
      turn_left(n_step);
      break;
    case CommandAction::Right:
      sender->GetSerial()->println("Turn right");
      if (!is_stand())
        stand();
      turn_right(n_step);
      break;
    case CommandAction::Shake:
      sender->GetSerial()->println("Hand shake");
      hand_shake(n_step);
      break;
    case CommandAction::Wave:
      sender->GetSerial()->println("Hand wave");
      hand_wave(n_step);
      break;
    case CommandAction::Test:
      sender->GetSerial()->println("Test");
      do_test();
      break;
    case CommandAction::StandSit:
      if (n_step)
      {
        sender->GetSerial()->println("Stand");
        stand();
      }
      else
      {
        sender->GetSerial()->println("Sit");
        sit();
      }
      break;
    default:
      sender->GetSerial()->println("Error");
      break;
  }
  sender->GetSerial()->println("..done.");
}

/*
  - setup callback for SerialCommand commands for Bluetooth module
   ---------------------------------------------------------------------------*/
SerialCommand cmd_action_("w", cmd_action);

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //start serial for debug and commands
  //Bluetooth default baud is 9600
  Serial1.begin(9600);
  CmdSerial.println("Robot starts initialization");

  // Initialise Detection Device
  switch (OBSTACLE_DETECT_DEVICE)
  {
    case ObstacleDetectorKind::None:
      break;
    case ObstacleDetectorKind::Infrared:
      // config IR_Detect_IO pin as input
      pinMode(IR_Detect_IO, INPUT);
      break;
    case ObstacleDetectorKind::Ultrasonic:
      // config Ultrasonic pins
      pinMode(Sonic_Detect_Trig, OUTPUT);
      pinMode(Sonic_Detect_Echo, INPUT);
      break;
  }

  SCmd.SetDefaultHandler(cmd_unrecognized);
  SCmd.AddCommand(&cmd_action_);

  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  CmdSerial.println("Servo service started");
  //initialize servos
  servo_attach();
  CmdSerial.println("Servos initialized");
  CmdSerial.println("Robot initialization Complete");
}

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

void servo_detach(void)
{
#if SERVO_VIA_PWM
  pwm.reset();
#else
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
  }
#endif
}

/*
  - loop function
   ---------------------------------------------------------------------------*/
int flag_obstacle = 0;
int mode_left_right = 0;
void loop()
{
  // SerialCommands read for Bluetooth module
  SCmd.ReadSerial();

  bool Obstacle = false;
  switch (OBSTACLE_DETECT_DEVICE)
  {
    case ObstacleDetectorKind::None:
      break;
    case ObstacleDetectorKind::Infrared:
      Obstacle = !digitalRead(IR_Detect_IO);
      break;
    case ObstacleDetectorKind::Ultrasonic:
      {
        // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
        digitalWrite(Sonic_Detect_Trig, LOW);
        delayMicroseconds(5);
        digitalWrite(Sonic_Detect_Trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(Sonic_Detect_Trig, LOW);

        // Read the signal from the sensor: a HIGH pulse whose
        // duration is the time (in microseconds) from the sending
        // of the ping to the reception of its echo off of an object.
        pinMode(Sonic_Detect_Echo, INPUT);
        long duration = pulseIn(Sonic_Detect_Echo, HIGH);

        // convert the time into a distance [centimetres]
        long dist = (duration / 2) / 29.1;

        // zero means nothing was detected
        Obstacle = dist > 0 && dist <= Sonic_Detect_Range;
      }
      break;
  }

  if (Obstacle && is_stand())
  {
    int tmp_turn = spot_turn_speed;
    int tmp_leg = leg_move_speed;
    int tmp_body = body_move_speed;
    spot_turn_speed = leg_move_speed = body_move_speed = 20;
    if (flag_obstacle < 3)
    {
      step_back(1);
      flag_obstacle++;
    }
    else
    {
      if (mode_left_right)
        turn_right(1);
      else
        turn_left(1);
      mode_left_right = 1 - mode_left_right;
      flag_obstacle = 0;
    }
    spot_turn_speed = tmp_turn;
    leg_move_speed = tmp_leg;
    body_move_speed = tmp_body;
  }
}

void do_test(void)
{
  CmdSerial.println("Stand");
  stand();
  delay(2000);
  CmdSerial.println("Step forward");
  step_forward(5);
  delay(2000);
  CmdSerial.println("Step back");
  step_back(5);
  delay(2000);
  CmdSerial.println("Turn left");
  turn_left(5);
  delay(2000);
  CmdSerial.println("Turn right");
  turn_right(5);
  delay(2000);
  CmdSerial.println("Hand wave");
  hand_wave(3);
  delay(2000);
  CmdSerial.println("Hand shake");
  hand_shake(3);
  delay(2000);
  CmdSerial.println("Sit");
  sit();
  delay(5000);
}

/*
  - default SerialCommands handler
  - gets called when no other command matches
   ---------------------------------------------------------------------------*/
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

/*
  - is_stand
  - return true if standing
   ---------------------------------------------------------------------------*/
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}

/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - body left
  - blocking function
  - parameter number of times to move body left
   ---------------------------------------------------------------------------*/
void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

/*
  - body right
  - blocking function
  - parameter number of times to move body right
   ---------------------------------------------------------------------------*/
void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

/*
  - hand wave
  - blocking function
  - parameter number of times to wave hand
   ---------------------------------------------------------------------------*/
void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

/*
  - hand shake
  - blocking function
  - parameter number of times to shake hand
   ---------------------------------------------------------------------------*/
void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected, this function moves the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site, to make sure the end point
   moves in a straight line, and decide the move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

/*
  - set one of end points' expect site
  - this function will set temp_speed[4][3] at the same time
  - non blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait until an end point moves to expected site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait until all end points move to expected site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
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
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  drive_servo(leg, 0, alpha);
  drive_servo(leg, 1, beta);
  drive_servo(leg, 2, gamma);
}
