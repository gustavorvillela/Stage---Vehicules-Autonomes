#!/usr/bin/env python3

from __future__ import division, print_function

import rospy
from std_msgs.msg import Float32, Int8, String,Int8MultiArray
from arduino_msgs.msg import Ardata
import sys


import signal
import time
import serial
import numpy as np
from time import sleep
import random

class Dialogue:

    def __init__(self, motor_speed=50, step_length=2):

    #===============================================================================================
    # Initialize ros publisher, ros subscriber
    #===============================================================================================
        
        # Topics where we publish
        self.motor_command = rospy.Publisher('raspi/motor',Int8MultiArray,queue_size=1)
        self.servo_command = rospy.Publisher('raspi/servo',Int8,queue_size=1)
        self.encoder_command = rospy.Publisher('raspi/encoder',Int8MultiArray,queue_size=1)
        self.data = rospy.Publisher('raspi/commands',Ardata,queue_size=1)

        # topics where to subscribe
        self.motor_sub = rospy.Subscriber("arduino/motor", Int8MultiArray, self.motorCallback)
        self.encoder_sub = rospy.Subscriber("arduino/encoder", Int8MultiArray, self.encoderCallback)
        
        # Actuators attributes
        self.motor_speed = motor_speed
        self.step_length = step_length
        self.encoder = Int8MultiArray()
        self.motor = Int8MultiArray()
        self.arduino = Arduino()


        # Communication attributes

        self.loss = []
        self.motor_listen = [self.motor_speed, self.motor_speed]
        self.encoder_listen = [0,0]

    #===============================================================================================
    # Callback functions
    #===============================================================================================
    
    # Motor callback
    def motorCallback(self,motor):

        self.motor_listen = motor.data
        right = self.motor_listen[0]
        left = self.motor_listen[1]
        if right != self.motor_speed:
            self.loss.append(right)
        elif left != self.motor_speed:
            self.loss.append(left)
        print("Right wheel: ",right)
        print("Left wheel: ",left,"\n")

    def encoderCallback(self,listen):

        self.encoder.data = listen.data

    #===============================================================================================
    # Motor command function
    #===============================================================================================

    def process_cmd(self,cmd):

        cmd_type = {
          "[q]uit": (cmd == 'q'),
          "[h]elp": (cmd == 'h'),
          "[e]ncoder values": (cmd == 'e'),
          "[z]ero setting encoders": (cmd == 'z'),
          "(%) set motor speed percentage": (cmd.isdigit()),
          "[f]orward step": (cmd == 'f'),
          "[l] left step": (cmd == 'l'),
          "[r] right step": (cmd == 'r'),
          "[b]ackward step": (cmd == 'b'),
          "[lb] left step back": (cmd == 'lb'),
          "[rb] right step back": (cmd == 'rb'),
          "[ff]orward": (cmd == 'ff'),
          "[bb]ackward": (cmd == 'bb'),
          "[tl] turn left": (cmd == 'tl'),
          "[tr] turn right": (cmd == 'tr'),
          "[p]ause motors": (cmd == 'p'),
          "[s]ervo move": (cmd == 's'),
          "[t]est motor msgs": (cmd == 't')
                   }
    
        if cmd_type["[q]uit"]:
            print("Goodbye...")
        elif cmd_type["[h]elp"]:
            for key in cmd_type.keys():
                print(key)
        elif cmd_type["[e]ncoder values"]:
            print('left encoder : ', self.encoder.data[1] )
            print('right encoder : ', self.encoder.data[0])
        elif cmd_type["[z]ero setting encoders"]:
            print("Resetting encoders...")
            self.encoder.data = [0,0]
            self.encoder_command.publish(self.encoder)
            print('left encoder : ', self.encoder.data[1])
            print('right encoder : ', self.encoder.data[0])
        elif cmd_type["(%) set motor speed percentage"]:
            self.motor_speed = int(cmd)
            print("Speed set to " + cmd + "%")
        elif cmd_type["[f]orward step"]:
            print("Moving forward at " + str(self.motor_speed) + "%...")
            self.motor.data = [self.motor_speed, self.motor_speed]
            self.motor_command.publish(self.motor)
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[l] left step"]:
            print("Forward left at " + str(self.motor_speed) + "%...")
            self.motor.data[0] = 0 #valeur moteur droit
            self.motor.data[1] = self.motor_speed #valeur moteur gauche
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[r] right step"]:
            print("Forward right at " + str(self.motor_speed) + "%...")
            self.motor.data[0] = self.motor_speed #valeur moteur droit
            self.motor.data[1] = 0 #valeur moteur gauche
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[b]ackward step"]:
            print("Moving backward at " + str(self.motor_speed) + "%...")
            self.motor.data = [-self.motor_speed, -self.motor_speed]
            self.motor_command.publish(self.motor)
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[lb] left step back"]:
            print("Backward left at " + str(self.motor_speed) + "%...")
            self.motor.data[0] = 0 #valeur moteur droit
            self.motor.data[1] = -self.motor_speed #valeur moteur gauche
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[rb] right step back"]:
            print("Backward right at " + str(self.motor_speed) + "%...")
            self.motor.data[0] = -self.motor_speed #valeur moteur droit
            self.motor.data[1] = 0 #valeur moteur gauche
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[ff]orward"]:
            print("Moving forward at " + str(self.motor_speed) + "%...")
            self.motor.data = [self.motor_speed, self.motor_speed]
            self.motor_command.publish(self.motor)
        elif cmd_type["[bb]ackward"]:
            print("Moving backward at " + str(self.motor_speed) + "%...")
            self.motor.data = [-self.motor_speed, -self.motor_speed]
            self.motor_command.publish(self.motor)
        elif cmd_type["[tl] turn left"]:
            print("Turn left at " + str(self.motor_speed) + "%...")
            self.motor.data[0] = self.motor_speed #valeur moteur droit
            self.motor.data[1] = -self.motor_speed #valeur moteur gauche
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[tr] turn right"]:
            print("Turn right at " + str(self.motor_speed) + "%...")
            self.motor.data[0] = -self.motor_speed #valeur moteur droit
            self.motor.data[1] = self.motor_speed #valeur moteur gauche
            time.sleep(self.step_length)
            print('stop motors')
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[p]ause motors"]:
            print("Stopping...")
            self.motor.data = [0,0]
            self.motor_command.publish(self.motor)
        elif cmd_type["[s]ervo move"]:
            print("Moving front servo...")
            self.servo_command.publish(45)
            time.sleep(2)
            self.servo_command.publish(90)
        elif cmd_type["[t]est motor msgs"]:

            print("Spinning motors...\n")
            self.loss = []
            self.motor_speed = 50
            total = 10
            rate = rospy.Rate(10)

            for i in range(total):

                self.motor.data = [self.motor_speed, self.motor_speed]
                self.arduino.motor = self.motor
                #self.motor_command.publish(self.motor)
                self.data.publish(self.arduino)
                self.motor_speed = self.motor_speed - 1
                rate.sleep()
    
            self.motor_speed = 0
            self.motor.data = [self.motor_speed, self.motor_speed]
            self.arduino.motor = self.motor
            #self.motor_command.publish(self.motor)
            self.data.publish(self.arduino)
            print("% of lost packets: " + str((1-len(self.loss)/total)*100) + "% for "+str(total)+" packets\n")
            rate.sleep()
        else:
            print("Invalid command")


#===============================================================================================
# Main function for input acquisition
#===============================================================================================

def main():

    diag = Dialogue()
    diag.motor.data = [diag.motor_speed, diag.motor_speed]
    diag.servo.data = 90
    diag.encoder.data = [0,0]

    print("Press enter to validate your commands")
    print("Enter h to get the list of valid commands")
    cmd_str = ''
    while cmd_str != 'q' and not rospy.is_shutdown():

        cmd_str = input("Enter your command: ")
        diag.process_cmd(cmd_str)


if __name__ == "__main__":
    try:
        rospy.init_node('raspi_ard_chat', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass