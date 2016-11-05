#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import ev3dev.ev3 as ev3
import math

import array
import fcntl
import sys

# from linux/input.h

KEY_UP = 103
KEY_DOWN = 108
KEY_LEFT = 105
KEY_RIGHT = 106
KEY_ENTER = 28
KEY_BACKSPACE = 14

KEY_MAX = 0x2ff

def EVIOCGKEY(length):
    return 2 << (14+8+8) | length << (8+8) | ord('E') << 8 | 0x18
    
BUF_LEN = (KEY_MAX + 7) / 8

def test_bit(bit, bytes):
    # bit in bytes is 1 when released and 0 when pressed
    return not bool(bytes[bit / 8] & (1 << (bit % 8)))

# default sleep timeout in sec
DEFAULT_SLEEP_TIMEOUT_IN_SEC = 0.05

# default threshold distance
DEFAULT_THRESHOLD_DISTANCE = 600 # 1000

##
# Setup
##

print("Setting up...")

# motors
right_motor = ev3.LargeMotor('outA')
print("motor right connected: %s" % str(right_motor.connected))

left_motor = ev3.LargeMotor('outB')
print("motor left connected: %s" % str(right_motor.connected))

motors = [left_motor, right_motor]
right_motor.reset()
left_motor.reset()

right_motor.speed_regulation_enabled = 'on'
left_motor.speed_regulation_enabled = 'on'

# sensors
color_sensor = ev3.ColorSensor()
print("color sensor connected: %s" % str(color_sensor.connected))
color_sensor.mode = 'COL-REFLECT'

ultrasonic_sensor = ev3.UltrasonicSensor()
print("ultrasonic sensor connected: %s" % str(ultrasonic_sensor.connected))
ultrasonic_sensor.mode = 'US-DIST-CM'

# default speed
DEFAULT_SPEED = 2000
SEARCH_SPEED = 400
SEARCH_SPEED_SLOW = 200

# print('search speed')
# print(str(SEARCH_SPEED))

##
#  Robot functionality
##
        
def revert():
    left_motor.stop()
    right_motor.stop()
   
    # new absolute position
    abs_pos_r = right_motor.position + 500
    abs_pos_l = left_motor.position + 500

    right_motor.position_sp = abs_pos_r
    right_motor.run_to_abs_pos()

    left_motor.position_sp = abs_pos_l
    left_motor.run_to_abs_pos()
    
    while abs(right_motor.position - abs_pos_r) > 10:
        pass

def forward():
    for m in motors:
        speed = m.speed_sp
        if speed > 0:
            m.speed_sp = speed * -1
        m.run_forever()


def set_speed(speed):
    for m in motors:
        m.speed_sp = -speed


def brake():
    for m in motors:
        m.stop()


def turn():
    left_motor.stop()
    pos = right_motor.position

    # new absolute position
    abs_pos = pos + 500

    right_motor.position_sp = abs_pos
    right_motor.run_to_abs_pos()

    while abs(right_motor.position - abs_pos) > 10:
        # turn to new position

        # stop when object detected
        if ultrasonic_sensor.value() < DEFAULT_THRESHOLD_DISTANCE:
            break

    set_speed(DEFAULT_SPEED)
    forward()

def search_turn(speed):
    left_motor.speed_sp = speed
    right_motor.speed_sp = -speed
    for m in motors:
        m.run_forever()     

def teardown():
    print('Tearing down...')
    for m in motors:
        m.stop()
        m.reset()       

def attack():
    set_speed(DEFAULT_SPEED)
    while True:
        time.sleep(DEFAULT_SLEEP_TIMEOUT_IN_SEC)
        if color_sensor.value() > 15:
            revert() 
            break
    
        if ultrasonic_sensor.value() < DEFAULT_THRESHOLD_DISTANCE:
            forward()
        else:
            for m in motors:
                m.stop()
            break
    
   
def search_slow():
    search_turn(-SEARCH_SPEED_SLOW)
    while True:
        time.sleep(DEFAULT_SLEEP_TIMEOUT_IN_SEC)
        if ultrasonic_sensor.value() < DEFAULT_THRESHOLD_DISTANCE:
            for m in motors:
                m.stop()
            break
   
  
def search_fast():        
    for m in motors:
        m.stop()
    search_turn(SEARCH_SPEED)
    while True:
        time.sleep(DEFAULT_SLEEP_TIMEOUT_IN_SEC)
        if ultrasonic_sensor.value() < DEFAULT_THRESHOLD_DISTANCE:
            for m in motors:
                m.stop()
            break
            
def search():
    search_fast()
    search_slow()
    
def run_loop():
    # game loop (endless loop)
    # found = False
    while True:
        print('color value: %s' % str(color_sensor.value()))
        print('ultrasonic value: %s' % str(ultrasonic_sensor.value()))
        print('motor positions (r, l): %s, %s' % (str(right_motor.position), str(left_motor.position)))
        
        attack()
        search()

            
def angle_to_pos(angle):
    # return angle / 360.0 * 650 - math.copysign(85, angle)
    return angle * 600 / 360.0            
            
def turn_angle_async(angle, speed):
    left_motor.speed_sp = speed
    right_motor.speed_sp = -speed
    
    # new absolute position
    abs_pos_r = right_motor.position + angle_to_pos(angle)
    abs_pos_l = left_motor.position - angle_to_pos(angle)

    right_motor.position_sp = abs_pos_r
    right_motor.run_to_abs_pos()

    left_motor.position_sp = abs_pos_l
    left_motor.run_to_abs_pos()

#angle is 0 to 360
def turn_angle(angle):
    left_motor.stop()
    right_motor.stop()
   
    # new absolute position
    abs_pos_r = right_motor.position + angle_to_pos(angle)
    abs_pos_l = left_motor.position - angle_to_pos(angle)

    right_motor.position_sp = abs_pos_r
    right_motor.run_to_abs_pos()

    left_motor.position_sp = abs_pos_l
    left_motor.run_to_abs_pos()
    
    while abs(right_motor.position - abs_pos_r) > 10:
        pass
    brake()
        
def main():
    print('Run robot, run!')
    # Wait for button press
    while True:
        buf = array.array('B', [0] * BUF_LEN)
        with open('/dev/input/by-path/platform-gpio-keys.0-event', 'r') as fd:
            ret = fcntl.ioctl(fd, EVIOCGKEY(len(buf)), buf)
        
        key = 'ENTER'
        key_code = globals()['KEY_' + key]
        key_state = test_bit(key_code, buf) and "pressed" or "released"
        if key_state == "pressed":
            break                  
    
    set_speed(DEFAULT_SPEED)
    # init
    try:
        turn_angle(180)
        search_slow()
        run_loop()
        
    # doing a cleanup action just before program ends
    # handle ctr+c and system exit
    except (KeyboardInterrupt, SystemExit):
        teardown()
        raise

    # handle exceptions
    except Exception as e:
        print('ohhhh error!')
        print(e)
        teardown()
        
##
# start the program
##
main()

# while True:
    # print('color value: %s' % str(color_sensor.value()))

