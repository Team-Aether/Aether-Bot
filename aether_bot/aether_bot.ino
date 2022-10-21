
// variable definitions

// global
int my_delay = 10;  // delay time // '-' optimum motor speed

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
int left_motor_speed = 45; // left motor runs at a slightly higher speed compared to the right motor
int turn_speed = 25;

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

int setpoint = 45;  // optimum motor speed

float kp = 300;  // proportional term
float kd = 400;  // derivative term
float ki = 0.00005;

// infrared sensors pins for line follower
#define RIGHT_IR_IN A1
#define LEFT_IR_IN A0

// obstacle avoidance - ultrasonic pins

// maze follower

// define states for state machine
enum state{
  LINE,
  STOP_GO,
  MAZE
};

// Variable to store the states
state current_state;
state previous_state;

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

  // initial state
  current_state = LINE;


}

void loop() {
  
  if(current_state == LINE){
    // run line follow
    follow_line();
    Serial.println("FOLLOWING LINE");
  } else if(current_state == STOP_GO){
    // run stop and go code
    Serial.println("STOP AND GO");
    // stop_and_go();
    
  }else if(current_state == MAZE){
    // run code for solving the maze
    Serial.println("SOLVING MAZE");
    // solve_maze();
  }
 
}

// function to call for line following state
void follow_line(){
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
    // delay(3000); // stop for 3 seconds then go // STOP AND GO
  } 
  else if((right_val == 0) && (left_val == 0)){
    // always move forward
    forward(right_motor_speed, left_motor_speed);
  } 
}


