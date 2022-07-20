#!/usr/bin/env python3
#########################################################################
# CENTRALESUPELEC : ST5 53 integration week
#
# Basic human-machine interface to test robots
#
#########################################################################
# Authors : Erwan Libessart
# Modifications by Morgan Roger
# TODO : translate to English where needed
#########################################################################

from __future__ import division, print_function

import rospy
from std_msgs.msg import Float32, Int8

import logging
import signal
import time
import serial
import numpy as np
from time import sleep
from picamera import PiCamera
import struct
import random

try:
    import queue
except ImportError:
    import Queue as queue

from robust_serial import write_order, Order, write_i8, write_i16, read_i16, read_i32, read_i8
from robust_serial.utils import open_serial_port
from constants import BAUDRATE

emptyException = queue.Empty
fullException = queue.Full
serial_file = None
camera = PiCamera()
motor_speed = 0
step_length = 1

lr_listen = motor_speed
ll_listen = motor_speed

right = rospy.Publisher('raspi_arduino_right',Int8, queue_size=10)
left = rospy.Publisher('raspi_arduino_left',Int8, queue_size=10)

def main():
    global motor_speed
    test_camera()
    #connect_to_arduino()
    print("Welcome to packet_loss.py")
    print("Press enter to validate your commands")
    print("Enter h to get the list of valid commands")
    cmd_str = ''
    while cmd_str != 'q' and not rospy.is_shutdown():
        right.publish(motor_speed)
        left.publish(motor_speed)
        cmd_str = input("Enter your command: ")
        process_cmd(cmd_str)
    camera.close()


def test_camera():
    global camera
    camera.start_preview()
    sleep(2)
    my_file = open('test_photo.jpg', 'wb')
    camera.capture(my_file)
    # At this point my_file.flush() has been called, but the file has
    # not yet been closed
    my_file.close()
    camera.stop_preview()


def connect_to_arduino():
    global serial_file
    try:
        # Open serial port (for communication with Arduino)
        serial_file = open_serial_port(serial_port='/dev/ttyACM0',baudrate=BAUDRATE)
    except Exception as e:
        print('exception')
        raise e

    is_connected = False
    # Initialize communication with Arduino
    while not is_connected:
        print("Trying connection to Arduino...")
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True

    time.sleep(2)
    c = 1
    while (c!=b''):
        c = serial_file.read(1)


def process_cmd(cmd):
    global motor_speed
    global step_length
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
        write_i8(serial_file, 0) #valeur moteur droit
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
        #refb = "{:016b}".format(ref)
        #ref = "General Kenobi !"
        #refb = ' '.join(format(i,'08b') for i in bytearray(ref,encoding='utf-8'))
        #refb = refb.replace(" ","")
        #print(refl," bits")
        print("Hello there...")
        for i in range(total):
            while True:
                ref = random.getrandbits(bits)
                if ref < 32767:
                    break
               # else:
               #     print(ref)
            #print("ref: ",ref)
            resp = listenArduino(ref)
            #respl = resp.bit_length()
            #print(resp)
            #respb = ' '.join(format(i,'08b') for i in bytearray(resp,encoding='utf-8'))
            #respb = respb.replace(" ","")
            #respb = "{:032b}".format(resp)
            if resp != ref:
               #bit_diff = refl - respl
               lresp.append(resp)
        print("% of lost packets: " + str(len(lresp)*100/total) + "% for "+str(total)+" packets")
        #if len(lresp) != 0:
        #    print("Mean bit lost: ",sum(lresp)/total)
    elif cmd_type["[t]est motor msgs"]:
        print("Spinning motors...\n")
        global lr_listen, ll_listen
        motor_speed=50
        loss = []
        total = 10
        for i in range(total):
            #motor_speed = motor_speed - 1
            right.publish(motor_speed)
            left.publish(motor_speed)
            rospy.Subscriber("arduino_raspi_right", Int8, callbackRight)
            rospy.Subscriber("arduino_raspi_left", Int8, callbackLeft)
            if lr_listen != motor_speed:
                loss.append(lr_listen)
            elif ll_listen != motor_speed:
                loss.append(ll_listen)
            time.sleep(0.1)
        motor_speed = 0
        right.publish(motor_speed)
        left.publish(motor_speed)
        print("% of lost packets: " + str(len(loss)*100/total) + "% for "+str(total)+" packets")
    else:
        print("Invalid command")


def lectureCodeurGauche():
    write_order(serial_file, Order.READENCODERl)
    while True:
       try:
           g = read_i16(serial_file)
           break
       except struct.error:
           pass
       except TimeoutError:
           write_order(serial_file, Order.READENCODERl)
           pass
    return g


def lectureCodeurDroit():
    write_order(serial_file, Order.READENCODERr)
    while True:
       try:
           d = read_i16(serial_file)
           break
       except struct.error:
           pass
       except TimeoutError:
           write_order(serial_file, Order.READENCODERr)
           pass
    return d

def listenArduino(numb):
    write_order(serial_file, Order.MESSAGE)
    ser = serial_file
    #ser.reset_input_buffer()
    count = 0
    m = 0
    while True:
       try:
           #ser.write(b"Hello there!\n")
           #m = ser.readline().decode('ascii').rstrip()
           write_i16(serial_file,numb)
           #print("Sent!")
           #av_trash = read_i32(serial_file)
           m = read_i16(serial_file)
           #print("resp: ",  m)
           #trash = read_i32(serial_file)
           time.sleep(0.01)
           break
       except TimeoutError:
           print("timeout!")
           write_order(serial_file, Order.MESSAGE)
           pass
       except struct.error:
           print("struct")
           pass
       except serial.serialutil.SerialException:
           pass
       except BlockingIOError:
           print("Blocked :P")
           pass
    return m

def testArduino(motor_speed):
    #print(motor_speed)
    #serial_file.reset_input_buffer()
    while True:
        try:
            write_order(serial_file, Order.TEST)
            write_i8(serial_file, motor_speed)
            write_i8(serial_file, motor_speed)
            lr = read_i8(serial_file)
            ll = read_i8(serial_file)
            sense = read_i16(serial_file)
            break
        except TimeoutError:
            write_order(serial_file, Order.TEST)
            pass
        except struct.error:
            print("Struct error")
            pass
        except serial.serialutil.SerialException:
            print("Serial Exception")
            pass
    return lr,ll

def callbackRight(lr):

    global lr_listen
    lr_listen = lr
    print("Right wheel: ",lr)



def callbackLeft(ll):

    global ll_listen
    ll_listen = ll
    print("Left wheel: ",ll,"\n")


if __name__ == "__main__":
    try:
        rospy.init_node('raspi_chat', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
