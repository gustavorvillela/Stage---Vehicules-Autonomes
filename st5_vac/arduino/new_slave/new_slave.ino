#include <Arduino.h>

#include "order.h"
#include "slave.h"
#include "parameters.h"
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#define USE_USBCON

#define LEFT 0
#define RIGHT 1

ros::NodeHandle_<ArduinoHardware, 25, 25, 200,360> nh;

std_msgs::Int8 right_wheel;
std_msgs::Int8 left_wheel;

ros::Publisher right("arduino_raspi_right", &right_wheel);
ros::Publisher left("arduino_raspi_left", &left_wheel);

void rightCB( const std_msgs::Int8 &right_speed){
  
  //motor_speed_right = right_speed.data;
  right_wheel.data = right_speed.data;
  
  
}  

void leftCB( const std_msgs::Int8 &left_speed){
  
  //motor_speed_left = left_speed.data;
  left_wheel.data = left_speed.data;
  
} 

ros::Subscriber<std_msgs::Int8> sub_right("raspi_arduino_right",rightCB);
ros::Subscriber<std_msgs::Int8> sub_left("raspi_arduino_left",leftCB);

void commCB( const std_msgs::String &comm){

  if (String(comm.data) == String("test"))
  {
    
    right.publish( &right_wheel );
    left.publish( &left_wheel );

  }
}


ros::Subscriber<std_msgs::String> command("comm",commCB);


void setup() {

  nh.getHardware()->setBaud(9600);

  nh.initNode();
  
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.subscribe(command);
  
  nh.advertise(right);
  nh.advertise(left);


  // Stop the car
  


}

void loop() {
  // put your main code here, to run repeatedly:
  
  //update_motors_orders();         
  
          
  nh.spinOnce();
  delay(1000);
}
