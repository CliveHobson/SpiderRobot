#pragma once

/* remote control ------------------------------------------------------------*/
#include <SerialCommands.h>

void servo_attach(void);

bool is_stand(void);
void stand(void);
void sit(void);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void hand_shake(int i);
void hand_wave(int i);
void do_test(void);

void wait_all_reach(void);
void set_site(int leg, float x, float y, float z);

void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
void polar_to_servo(int leg, float alpha, float beta, float gamma);

void servo_service(void);

void cmd_unrecognized(SerialCommands* sender, const char* cmd);