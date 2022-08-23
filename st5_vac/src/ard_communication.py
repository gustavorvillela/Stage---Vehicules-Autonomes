#!/usr/bin/env python3

from __future__ import division, print_function

import rospy
from std_msgs.msg import Float32, Int8, String,Int8MultiArray
import sys


import signal
import time
import serial
import numpy as np
from time import sleep
import random

class Dialogue:

    def __init__(self, motor_speed=0, step_length=1):

    #===============================================================================================
    # Initialize ros publisher, ros subscriber
    #===============================================================================================
        
        # Topics where we publish
        self.motor_command = rospy.Publisher('raspi_arduino',Int8MultiArray,queue_size=10)
        self.command = rospy.Publisher('comm',String,queue_size=10)

        # topics where to subscribe
        self.arduino_sub = rospy.Subscriber("arduino_raspi", Int8MultiArray, motorCallback)
        
        # Motor attributes
        self.motor_speed = motor_speed
        self.step_length = step_length

        # Communication attributes

        self.loss = []
        self.lr_listen = self.motor_speed
        self.ll_listen = self.motor_speed
        self.sent = Int8MultiArray()

    #===============================================================================================
    # Callback functions
    #===============================================================================================
    
    # Motor callback
    def motorCallback(listen):

        global sent,loss
        self.lr_listen = listen.data[0]
        self.ll_listen = listen.data[1]
        if self.lr_listen != self.motor_speed:
            self.loss.append(self.lr_listen)
        elif self.ll_listen != self.motor_speed:
            self.loss.append(self.ll_listen)
        print("Right wheel: ",self.lr_listen)
        print("Left wheel: ",self.ll_listen,"\n")


    #===============================================================================================
    # Motor command function
    #===============================================================================================

    def process_cmd(cmd):

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
          "[m]essage arduino": (cmd == 'm'),
          "[t]est motor msgs": (cmd == 't')
                   }
    
        if cmd_type["[q]uit"]:
            print("Goodbye...")
        elif cmd_type["[h]elp"]:
            for key in cmd_type.keys():
                print(key)
        elif cmd_type["[e]ncoder values"]:
            print('left encoder : ', lectureCodeurGauche())
            print('right encoder : ', lectureCodeurDroit())
        elif cmd_type["[z]ero setting encoders"]:
            print("Resetting encoders...")
            write_order(serial_file, Order.RESETENC)
            print('left encoder : ', lectureCodeurGauche())
            print('right encoder : ', lectureCodeurDroit())
        elif cmd_type["(%) set motor speed percentage"]:
            motor_speed = int(cmd)
            print("Speed set to " + cmd + "%")
        elif cmd_type["[f]orward step"]:
            print("Moving forward at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, motor_speed) #valeur moteur droit
            write_i8(serial_file, motor_speed) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[l] left step"]:
            print("Forward left at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, 0) #valeur moteur droit
            write_i8(serial_file, motor_speed) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[r] right step"]:
            print("Forward right at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, motor_speed) #valeur moteur droit
            write_i8(serial_file, 0) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[b]ackward step"]:
            print("Moving backward at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, -motor_speed) #valeur moteur droit
            write_i8(serial_file, -motor_speed) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[lb] left step back"]:
            print("Backward left at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, 0) #valeur moteur droit0.
            write_i8(serial_file, -motor_speed) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[rb] right step back"]:
            print("Backward right at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, -motor_speed) #valeur moteur droit
            write_i8(serial_file, 0) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[ff]orward"]:
            print("Moving forward at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, motor_speed) #valeur moteur droit
            write_i8(serial_file, motor_speed) #valeur moteur gauche
        elif cmd_type["[bb]ackward"]:
            print("Moving backward at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, -motor_speed) #valeur moteur droit
            write_i8(serial_file, -motor_speed) #valeur moteur gauche
        elif cmd_type["[tl] turn left"]:
            print("Turn left at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, motor_speed) #valeur moteur droit
            write_i8(serial_file, -motor_speed) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[tr] turn right"]:
            print("Turn right at " + str(motor_speed) + "%...")
            write_order(serial_file, Order.MOTOR)
            write_i8(serial_file, -motor_speed) #valeur moteur droit
            write_i8(serial_file, motor_speed) #valeur moteur gauche
            time.sleep(step_length)
            print('stop motors')
            write_order(serial_file, Order.STOP)
        elif cmd_type["[p]ause motors"]:
            print("Stopping...")
            write_order(serial_file, Order.STOP)
        elif cmd_type["[s]ervo move"]:
            print("Moving front servo...")
            write_order(serial_file, Order.SERVO)
            write_i16(serial_file, 45) #valeur angle servo
            time.sleep(2)
            write_order(serial_file, Order.SERVO)
            write_i16(serial_file, 90) #valeur angle servo
        elif cmd_type["[m]essage arduino"]:
            lresp = []
            total = 100
            bits = 16
            print("Hello there...")
            for i in range(total):
                while True:
                    ref = random.getrandbits(bits)
                    if ref < 32767:
                        break
                resp = listenArduino(ref)
                if resp != ref:
                   lresp.append(resp)
            print("% of lost packets: " + str(len(lresp)*100/total) + "% for "+str(total)+" packets")
        elif cmd_type["[t]est motor msgs"]:

            print("Spinning motors...\n")
            self.loss = []
            self.motor_speed = 50
            total = 10
            rate = rospy.Rate(10)

            for i in range(total):

                self.command.publish("test")
                self.sent.data = [self.motor_speed, self.motor_speed]
                self.motor_command.publish(self.sent)
                self.motor_speed = self.motor_speed - 1

                rate.sleep()
    
            self.motor_speed = 0
            self.sent.data = [0,0]
            self.motor_command.publish(self.sent)
            print("% of lost packets: " + str(len(self.loss)*100/total) + "% for "+str(total)+" packets\n")
            self.command.publish("stop")
            rate.sleep()
        else:
            print("Invalid command")


def main():

    diag = Dialogue()

    print("Press enter to validate your commands")
    print("Enter h to get the list of valid commands")
    cmd_str = ''
    while cmd_str != 'q' or not rospy.is_shutdown():

        cmd_str = input("Enter your command: ")
        diag.process_cmd(cmd_str)


if __name__ == "__main__":
    try:
        rospy.init_node('raspi_ard_chat', anonymous=True)
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass