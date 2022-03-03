#!/usr/bin/env python

# For running in RPi4
# Listening to ROS topic for pump, solenoid and motor commands

import rospy
from std_msgs.msg import String, Empty, Float64, Bool, Int8
from multiprocessing import Value
from ctypes import c_bool, c_float

import time
import RPi.GPIO as GPIO            # import RPi.GPIO module  

# define GPIO pin
PUMP = 23
SOLENOID = 24

# for winch
EN = 10
DIR = 9
PWM = 11

SERVO_L = 18
SERVO_R = 19

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

def ReceiveWinchMessage(data):
    if winch_state.value != data.data:
        winch_state.value = data.data
        trigger_winch.value = True

    #rospy.loginfo("Current motor state = " + str(winch_state.value))

def ReceiveServoMessage(data):
    if servo_state.value != data.data:
        servo_state.value = data.data
        trigger_servo.value = True
        
    rospy.loginfo("current servo state = {0}".format(servo_state.value))

def winch(action):
    if action == 'up':
        rospy.loginfo("winch up")
        GPIO.output(EN, GPIO.LOW)
        GPIO.output(DIR, GPIO.LOW)
        GPIO.output(PWM, GPIO.HIGH)
    elif action == 'down':
        rospy.loginfo("winch backward")
        GPIO.output(EN, GPIO.LOW)
        GPIO.output(DIR, GPIO.HIGH)
        GPIO.output(PWM, GPIO.HIGH)
    elif action == 'stop':
        rospy.loginfo("winch stop")
        GPIO.output(EN, GPIO.LOW)
        GPIO.output(DIR, GPIO.LOW)
        GPIO.output(PWM, GPIO.LOW)

def servo(action):
    if action == 'open':
        rospy.loginfo("Opening up servo latching")
        for duty in range(0,101,1):
            l_motor_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
            r_motor_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
            rospy.sleep(0.01)
    elif action == 'close':        
        rospy.loginfo("Closing down servo latching")
        for duty in range(100,-1,-1):
            l_motor_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
            r_motor_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
            rospy.sleep(0.01)

if __name__ == '__main__':
    global pump_state
    global solenoid_state
    global winch_state
    global trigger_winch
    global servo_state
    global trigger_servo
    
    GPIO.setwarnings(False)	
    GPIO.setmode(GPIO.BCM)               # choose BCM or BOARD  
    GPIO.setup(PUMP, GPIO.OUT)           
    GPIO.setup(SOLENOID, GPIO.OUT)
    GPIO.setup(SERVO_L, GPIO.OUT)
    GPIO.setup(SERVO_R, GPIO.OUT)
    
    GPIO.output(PUMP, GPIO.LOW)
    GPIO.output(SOLENOID, GPIO.LOW)
    
    l_motor_pwm = GPIO.PWM(SERVO_L, 1000)
    r_motor_pwm = GPIO.PWM(SERVO_R, 1000)
    l_motor_pwm.start(0)
    r_motor_pwm.start(0)
    
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(PWM, GPIO.OUT)

    # stop the winch at the beginning
    GPIO.output(EN, GPIO.LOW)
    GPIO.output(DIR, GPIO.LOW)
    GPIO.output(PWM, GPIO.LOW)

    pump_state = Value(c_bool, False)
    solenoid_state = Value(c_bool, False)
    winch_state = Value(c_float, 0)
    trigger_winch = Value(c_bool, False)
    servo_state = Value(c_bool, False)
    trigger_servo = Value(c_bool, False)
    
    rospy.init_node('Electronic_Node')
    rate = rospy.Rate(10) # 10hz

    subPump = rospy.Subscriber('pump_on', Empty, ReceivePumpMessage)
    subSolenoid = rospy.Subscriber('solenoid_on', Empty, ReceiveSolenoidMessage)
    subWinch = rospy.Subscriber('winch_state', Float64, ReceiveWinchMessage)
    subServos = rospy.Subscriber('servo_state', Bool, ReceiveServoMessage)

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

            rospy.loginfo("Get winch state = {0}".format(winch_state.value))         
            
            if trigger_winch.value:
                if winch_state.value > 0 : # motor runs forward
                    winch('up')
                elif winch_state.value == 0: # motor stop
                    winch('stop')
                elif winch_state.value < 0: # motor runs backward
                    winch('down')
                trigger_winch.value = False
            
            if trigger_servo.value:
                if servo_state.value:
                    servo('open')
                else:
                    servo('close')
                trigger_servo.value = False


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
        trigger_winch.value = False   
        trigger_servo.value = False
        GPIO.cleanup()
