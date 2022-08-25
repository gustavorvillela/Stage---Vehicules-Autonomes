#include <Arduino.h>

//Importing important libraries
#include "order.h"
#include "slave.h"
#include "parameters.h"
#include <Servo.h>
#include <ros.h> //ros_lib
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/String.h>
#include <arduino_msgs/Ardata.h>
#define USE_USBCON

//********************
// Defining variables
//********************

#define LEFT 0
#define RIGHT 1

Servo frontServo, backServo;    // create servo objects
#define  frontPin 9
#define  backPin 8
int pos = 90;    // initial position of the servos
int sweepFlag = 1;
int coder[2] = {
  0,0};
int lastSpeed[2] = {
  0,0};
int8_t motor_speed_right = 0;
int8_t motor_speed_left = 0;


//************************
// Defining ROS variables
//************************

ros::NodeHandle nh; //ROS node

//Starting messages
std_msgs::Int8MultiArray wheels;
std_msgs::Int8MultiArray encoder;
std_msgs::Int8 servo;

//Defining publisher objects
ros::Publisher mot("/arduino/motor", &wheels);
ros::Publisher enc("/arduino/encoder", &encoder);

//************************
// Callback functions
//************************

// Motor callback
//void motorCB( const std_msgs::Int8MultiArray &motor_speed){
//  
//  wheels = motor_speed;
//  mot.publish( &wheels );
//  motor_speed_right = motor_speed.data[0];
//  motor_speed_left = motor_speed.data[1];
//  
//} 
//
//// Servo callback
//void servoCB( const std_msgs::Int8 &servo_angle){
//
//  pos = servo_angle.data;
//  frontServo.write(pos);
//  
//} 
//
//
//
//// Encoder callback
//void encoderCB( const std_msgs::Int8MultiArray &encoder_speed){
//
//  int8_t right;
//  int8_t left;
//  right = encoder_speed.data[0];
//  left = encoder_speed.data[1];
//
//  coder[LEFT] = left;
//  coder[RIGHT] = right;
//
//  
//  
//} 

void commCB( const arduino_msgs::Ardata &arduino){

  int8_t right;
  int8_t left;

  if (arduino.command == "motor")
  {
    //Motor callback
  
    wheels = arduino.motor;
    mot.publish( &wheels );
    motor_speed_right = arduino.motor.data[0];
    motor_speed_left = arduino.motor.data[1];
    
  }
  else if (arduino.command == "encoder")
  {
    //Encoder callback
    right = arduino.encoder.data[0];
    left = arduino.encoder.data[1];

    coder[LEFT] = left;
    coder[RIGHT] = right;
  }
  else if (arduino.command == "servo")
  {
    //Servo callback
    pos = arduino.servo.data;
    frontServo.write(pos);
  }
  
} 

//Defining subscriber objects
//ros::Subscriber<std_msgs::Int8MultiArray> motor_sub("/raspi/motor",motorCB);
//ros::Subscriber<std_msgs::Int8MultiArray> enc_sub("/raspi/encoder",encoderCB);
//ros::Subscriber<std_msgs::Int8> servo_sub("/raspi/servo",servoCB);
ros::Subscriber<arduino_msgs::Ardata> arduino_sub("/raspi/commands",commCB);


void setup() {

  nh.getHardware()->setBaud(57600);

  nh.initNode();
  
//  nh.subscribe(motor_sub);
//  nh.subscribe(enc_sub);
//  nh.subscribe(servo_sub);
  nh.subscribe(arduino_sub);
  
  nh.advertise(mot);
  nh.advertise(enc);

  frontServo.attach(frontPin);
  frontServo.write(pos);
  backServo.attach(backPin);
  backServo.write(pos);

  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3

  // Init Motor
  pinMode(SPEED_LEFT, OUTPUT);
  pinMode(SPEED_RIGHT, OUTPUT);
  pinMode(DIRECTION_LEFT, OUTPUT);
  pinMode(DIRECTION_RIGHT, OUTPUT);
  // Stop the car
  stop();
  
}


void loop() {
  
  update_motors_orders();
  update_encoder();        
  nh.spinOnce();
  delay(20);
}

void update_encoder()
{
  encoder.data[RIGHT] = coder[RIGHT];
  encoder.data[LEFT] = coder[LEFT];

  enc.publish( &encoder );
}

void update_motors_orders()
{

  motor_speed_right = constrain(motor_speed_right, -SPEED_MAX, SPEED_MAX);
  motor_speed_left = constrain(motor_speed_left, -SPEED_MAX, SPEED_MAX);


  if (motor_speed_left > 0)
  {
    digitalWrite(DIRECTION_LEFT, LOW);
  }
  else
  {
    digitalWrite(DIRECTION_LEFT, HIGH);
  }
  
    if (motor_speed_right > 0)
  {
    digitalWrite(DIRECTION_RIGHT, HIGH);
  }
  else
  {
       digitalWrite(DIRECTION_RIGHT, LOW);
  }
  analogWrite(SPEED_LEFT, convert_to_pwm(float(motor_speed_left)));
  analogWrite(SPEED_RIGHT, convert_to_pwm(float(motor_speed_right)));
}

void stop()
{

  analogWrite(SPEED_LEFT, 0);
  digitalWrite(DIRECTION_LEFT, LOW);
  analogWrite(SPEED_RIGHT, 0);
  digitalWrite(DIRECTION_RIGHT, LOW);  
}

int convert_to_pwm(float motor_speed)
{
  // TODO: compensate the non-linear dependency speed = f(PWM_Value)
  return (int) round(abs(motor_speed)*(255./100.));
}


void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}
