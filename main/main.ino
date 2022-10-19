
// variable definitions

// global
int my_delay = 10;  // delay time
int magic_number = 40; // '-' optimum motor speed

// motor pins
#define in1 7 // left motor
#define in2 8

#define in3 12 // right motor
#define in4 2 

#define ena 9 // left motor
#define enb 11 // right motor 

// speed constants
int base_speed = 45; // speed when the robot is moving forward
int right_motor_speed = 38; // sped of the motors when they are running on a straight line 
int left_motor_speed = 38; // left motor runs at a slightly higher speed compared to the right motor
int turn_speed = 15;

// motor movement function prototypes
void forward(int, int);
void left(int, int);
void right(int, int);
void reverse(int, int);
void stop();

// pid
float PID(double);
unsigned long current_time, previous_time;
double elapsed_time;
double error;
double last_error;
double input, output, set_point;
double cumulative_error, rate_error;

int setpoint = magic_number;  // optimum motor speed

float kp = 450;  // proportional term
float kd = 400;  // derivative term
float ki = 0.000005;

// infrared sensors pins for line follower
#define RIGHT_IR_IN A1
#define LEFT_IR_IN A0
int follow_line();

// obstacle avoidance - ultrasonic pins

// maze follower

int return_val; // this variable advises our state machine implementation

void setup() {
  Serial.begin(9600);

  // set up motor pins
  pinMode(RIGHT_IR_IN, INPUT);
  pinMode(LEFT_IR_IN, INPUT);

  // set up motor driver pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
}

void loop() {

  // =============== Sequence m
  
  // 1. Call line following first
//  return_val = follow_line();
//
//  if(return_val == 1){
//    // line following is done, call obstacle avoidance
//    return_val = obstacle_avoidance();
//    if(return_val == 1){
//      // obstacle avoindance is done, call maze solver
//      return_val = maze_solver();
//      if(return_val == 1){
//        // stop: we are done with the game
//        stop();
//      }
//    }
//  }

  follow_line();
 
}

/*
Implement line follower functions
*/

int follow_line(){
  int right_val = digitalRead(RIGHT_IR_IN);
  int left_val = digitalRead(LEFT_IR_IN);

  right_motor_speed = right_motor_speed - PID(right_motor_speed); // apply correction to the right motor
  left_motor_speed = left_motor_speed + PID(left_motor_speed);  // apply correction to the left motor
   
  if((right_val == 1) && (left_val == 0)){
    // steer to the right
    right(right_motor_speed, left_motor_speed);
  }
  
  else if((right_val == 0) && (left_val == 1)){
    // steer to the left
    left(right_motor_speed, left_motor_speed);
  }
 
  else if((right_val == 1) && (left_val == 1)){
    // stop at a t-junction
    stop();
    delay(3000); // stop for 3 seconds then go // STOP AND GO
  } 
  else if((right_val == 0) && (left_val == 0)){
    // always move forward
    // forward(right_motor_speed, left_motor_speed);

    return 1; // means we are done with the line following part    
  } 

}

// ==============================Motor functions ====================================
void forward(int r_speed, int l_speed){

  // turn ON all motors  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  

  // left motor on 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(ena, l_speed);
  analogWrite(enb, r_speed);

}

void left(int r_speed, int l_speed){
  // turn motors left - right motor on, left motor reversed direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  

  analogWrite(ena, turn_speed);
  analogWrite(enb, r_speed);

}

void right(int r_speed, int l_speed){
  // turn motors right
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  

  analogWrite(ena, l_speed);
  analogWrite(enb, turn_speed);
}

void stop(){
  // turn motors OFF
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
}

//===================PID=================

float PID(double inp){
  /*
  * calculate the correction to be applied to the motor movement
  */
  current_time = millis();
  elapsed_time = (double)(current_time - previous_time);

  error = setpoint - inp;
  cumulative_error += error * elapsed_time;
  rate_error = (error - last_error) / elapsed_time;

  double out = kp*error + ki*cumulative_error + kd*rate_error;

  last_error = error;
  previous_time = current_time;

  return output;
 
}
