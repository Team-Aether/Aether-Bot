#ifndef AETHER_BOT_H
#define AETHER_BOT_H

/*Infara-red sensors*/

#define L_IR_IN 2
#define R_IR_IN 3

/*Motor Connections*/

#define IN1   4
#define IN2   5
#define IN3   6
#define IN4   7
#define ENA   10
#define ENB   11

#define IRS 12
#define IRR 13
#define IRL 14

const int speed = 45;
const int left_motor_speed = 30;
const int turn_speed = 20;
const int my_delay = 50;

void forward();
void reverse();
void left();
void right();
void stop();

#endif
