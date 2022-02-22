#!/usr/bin/env python

# For running in RPi4
# Listening to ROS topic for pump, solenoid and motor commands

import rospy
from std_msgs.msg import String, Empty, Float64, Bool, Int8
from multiprocessing import Value
from ctypes import c_bool, c_int

import time
import RPi.GPIO as GPIO            # import RPi.GPIO module  

# define GPIO pin
PUMP = 23
SOLENOID = 24

# for winch
EN = 10
DIR = 9
PWM = 11

motor_state = 0

def ReceivePumpMessage(data):
    if pump_state.value:
        pump_state.value = False
        rospy.loginfo("Turn pump off")
    else:
        pump_state.value = True
        rospy.loginfo("Turn pump on")

def ReceiveSolenoidMessage(data):
    if solenoid_state.value:
        solenoid_state.value = False
        rospy.loginfo("Turn Solenoid off")
    else:
        solenoid_state.value = True
        rospy.loginfo("Turn Solenoid on")

def ReceiveMotorMessage(data):
    global motor_state
    if data == 0:
        motor_state = 0
    elif data == 1:
        motor_state = 1
    elif data == -1:
        motor_state = -1    
    #motor_state.value = int(data)
    rospy.loginfo("Current winch state = {0}".format(motor_state))


if __name__ == '__main__':
    global subPump
    global subSolenoid
    global pump_state
    global solenoid_state
    
    GPIO.setmode(GPIO.BCM)               # choose BCM or BOARD  
    GPIO.setup(PUMP, GPIO.OUT)           
    GPIO.output(PUMP, GPIO.LOW)
    GPIO.setup(SOLENOID, GPIO.OUT)
    GPIO.output(SOLENOID, GPIO.LOW)
    
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(PWM, GPIO.OUT)

    GPIO.output(EN, GPIO.LOW)
    GPIO.output(DIR, GPIO.LOW)
    GPIO.output(PWM, GPIO.LOW)

    pump_state = Value(c_bool, False)
    solenoid_state = Value(c_bool, False)
    
    rospy.init_node('Electronic_Node')
    rate = rospy.Rate(20) # 10hz

    subPump = rospy.Subscriber('pump_on', Empty, ReceivePumpMessage)
    subSolenoid = rospy.Subscriber('solenoid_on', Empty, ReceiveSolenoidMessage)
    subWinch = rospy.Subscriber('winch_state', Int8, ReceiveMotorMessage)

    try:
        while not rospy.is_shutdown():
            if pump_state.value:
                GPIO.output(PUMP, GPIO.HIGH)
            else:
                GPIO.output(PUMP, GPIO.LOW)

            if solenoid_state.value:
                GPIO.output(SOLENOID, GPIO.HIGH)
            else:
                GPIO.output(SOLENOID, GPIO.LOW)

            rospy.loginfo("Get winch state = {0}".format(motor_state))         
            if motor_state == 1 : # motor runs forward
                GPIO.output(EN, GPIO.LOW)
                GPIO.output(DIR, GPIO.LOW)
                GPIO.output(PWM, GPIO.HIGH)
            elif motor_state == 0: # motor stop
                GPIO.output(EN, GPIO.LOW)
                GPIO.output(DIR, GPIO.LOW)
                GPIO.output(PWM, GPIO.LOW)
            elif motor_state == -1: # motor runs backward
                GPIO.output(EN, GPIO.LOW)
                GPIO.output(DIR, GPIO.HIGH)
                GPIO.output(PWM, GPIO.HIGH)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupts")
        
    finally:
        rospy.loginfo("Node stops. Bye-bye!")
        GPIO.output(PUMP, GPIO.LOW)
        GPIO.output(SOLENOID, GPIO.LOW)  
        GPIO.output(EN, GPIO.LOW)
        GPIO.output(DIR, GPIO.LOW)
        GPIO.output(PWM, GPIO.LOW)      
        GPIO.cleanup()
