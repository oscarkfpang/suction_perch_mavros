#!/usr/bin/env python

# For running in RPi4
# Listening to ROS topic for pump command

import rospy
import tf
from std_msgs.msg import String, Empty, Float64, Bool
from multiprocessing import Value
from ctypes import c_bool
from sensor_msgs.msg import Joy

import time

import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay  

# define GPIO pin
PUMP = 18
SOLENOID = 15

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



if __name__ == '__main__':
    global subPump
    global subSolenoid
    global pump_state
    global solenoid_state
    
    GPIO.setmode(GPIO.BCM)               # choose BCM or BOARD  
    GPIO.setup(PUMP, GPIO.OUT)           
    GPIO.output(PUMP, 0)
    GPIO.setup(SOLENOID, GPIO.OUT)
    GPIO.output(SOLENOID, 0)

    pump_state = Value(c_bool, False)
    solenoid_state = Value(c_bool, False)
    
    rospy.init_node('Electronic Node')
    rate = rospy.Rate(20) # 10hz

    subPump = rospy.Subscriber('pump_on', Empty, ReceivePumpMessage)
    subSolenoid = rospy.Subscriber('solenoid_on', Empty, ReceiveSolenoidMessage)

    try:
        while not rospy.is_shutdown():
            if pump_state.value:
                GPIO.output(PUMP, 1)
            else:
                GPIO.output(PUMP, 0)

            if solenoid_state.value:
                GPIO.output(SOLENOID, 1)
            else:
                GPIO.output(SOLENOID, 0)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupts")
        
    finally:
        rospy.loginfo("Node stops. Bye-bye!")
        GPIO.output(PUMP, 0)
        GPIO.output(SOLENOID, 0)        
        GPIO.cleanup()
