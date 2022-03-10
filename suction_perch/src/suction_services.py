#!/usr/bin/env python

# For running in RPi4
# Listening to ROS topic for pump, solenoid and motor commands

import rospy
from std_msgs.msg import String, Empty, Float64, Bool, Int8
from multiprocessing import Value
from ctypes import c_bool, c_float

import time
import RPi.GPIO as GPIO            # import RPi.GPIO module  
import pigpio

# define GPIO pin
PUMP = 23
SOLENOID = 24

# for winch
EN = 10
DIR = 9
PWM = 11

SERVO_L = 18
SERVO_R = 19

SWITCH = 17

L_OPEN_DC = 90
L_CLOSE_DC = 97
R_OPEN_DC = 62
R_CLOSE_DC = 56

FREQ = 50


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
    #if override.value:
    #    override_cmd.value = data.data
    #    return

    if winch_state.value != data.data:
        winch_state.value = data.data
        trigger_winch.value = True

    #rospy.loginfo("Current motor state = " + str(winch_state.value))

def ReceiveServoMessage(data):
    if servo_state.value != data.data:
        servo_state.value = data.data
        trigger_servo.value = True
        
    #rospy.loginfo("current servo state = {0}".format(servo_state.value))

def ReceiveOverrideMessage(data):
    override.value = data.data

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
        '''
        for duty in range(0, 11): 
            l_motor_pwm.ChangeDutyCycle(duty*5+50) #provide duty cycle in the range 0-100
            r_motor_pwm.ChangeDutyCycle(100-duty*5) #provide duty cycle in the range 0-100
        '''                
        pwm.hardware_PWM(SERVO_L, FREQ, L_OPEN_DC*1000)
        pwm.hardware_PWM(SERVO_R, FREQ, R_OPEN_DC*1000)

    elif action == 'close':        
        rospy.loginfo("Closing down servo latching")
        '''
        for duty in range(0, 11):
            l_motor_pwm.ChangeDutyCycle(100-duty*5) #provide duty cycle in the range 0-100
            r_motor_pwm.ChangeDutyCycle(duty*5+50) #provide duty cycle in the range 0-100  
            rospy.sleep(0.05)
        '''
        pwm.hardware_PWM(SERVO_L, FREQ, L_CLOSE_DC*1000)
        pwm.hardware_PWM(SERVO_R, FREQ, R_CLOSE_DC*1000)
        
                
if __name__ == '__main__':
    global pump_state
    global solenoid_state
    global winch_state
    global trigger_winch
    global servo_state
    global trigger_servo
    
    global override
    global override_cmd
    
    global pwm
    
    GPIO.setwarnings(False)	
    GPIO.setmode(GPIO.BCM)               # choose BCM or BOARD  
    GPIO.setup(PUMP, GPIO.OUT)           
    GPIO.setup(SOLENOID, GPIO.OUT)
    GPIO.setup(SERVO_L, GPIO.OUT)
    GPIO.setup(SERVO_R, GPIO.OUT)
    GPIO.setup(SWITCH, GPIO.IN)  
    
    GPIO.output(PUMP, GPIO.LOW)
    GPIO.output(SOLENOID, GPIO.LOW)
    
    #l_motor_pwm = GPIO.PWM(SERVO_L, 500)
    #r_motor_pwm = GPIO.PWM(SERVO_R, 500)
    #l_motor_pwm.start(0)
    #r_motor_pwm.start(0)
    
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(PWM, GPIO.OUT)

    # stop the winch at the beginning
    GPIO.output(EN, GPIO.LOW)
    GPIO.output(DIR, GPIO.LOW)
    GPIO.output(PWM, GPIO.LOW)
    
    pwm = pigpio.pi() 
    pwm.set_mode(SERVO_L, pigpio.OUTPUT)
    pwm.set_mode(SERVO_R, pigpio.OUTPUT)
    pwm.set_PWM_frequency(SERVO_L, FREQ)
    pwm.set_PWM_frequency(SERVO_R, FREQ)

    pump_state = Value(c_bool, False)
    solenoid_state = Value(c_bool, False)
    winch_state = Value(c_float, 0)
    trigger_winch = Value(c_bool, False)
    servo_state = Value(c_bool, False)
    trigger_servo = Value(c_bool, False)
    override = Value(c_bool, False)
    override_cmd = Value(c_float, 0)
    prev_switch_state = 1
    
    contact = Value(c_bool, False)  ## initialise this to True in production
    
    rospy.init_node('Electronic_Node')
    rate = rospy.Rate(50) # 50hz

    subPump = rospy.Subscriber('pump_on', Empty, ReceivePumpMessage)
    subSolenoid = rospy.Subscriber('solenoid_on', Empty, ReceiveSolenoidMessage)
    subWinch = rospy.Subscriber('winch_state', Float64, ReceiveWinchMessage)
    subServos = rospy.Subscriber('servo_state', Bool, ReceiveServoMessage)
    subOverride = rospy.Subscriber('override', Bool, ReceiveOverrideMessage)

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
            
            if not override.value and trigger_winch.value:
                if winch_state.value > 0 : # winch up
                    override.value = False
                    winch('up')
                elif winch_state.value == 0: # motor stop
                    winch('stop')
                elif winch_state.value < 0: # winch down
                    winch('down')
                trigger_winch.value = False

            if not override.value and trigger_servo.value:
                if servo_state.value:
                    servo('open')
                else:
                    servo('close')
                trigger_servo.value = False
            
            if override.value: ## and trigger_winch.value:
                if winch_state.value > 0: # to lower the sensor and open the latch
                    winch('up)    
                if GPIO.input(SWITCH): 
                    rospy.loginfo("Switch is pressed! Contact!")     
                    # close the latch and save the existing latch value
                    contact.value = 1
                    rospy.sleep(0.05)
                    servo('close')
                    rospy.loginfo("Latch is closed!")     
                        
                    


            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupts")
        
    finally:
        rospy.loginfo("Node stops. Bye-bye!")
        pwm.set_servo_pulsewidth(SERVO_L,0)
        pwm.set_servo_pulsewidth(SERVO_R,0)
        pwm.stop()

        GPIO.output(PUMP, GPIO.LOW)
        GPIO.output(SOLENOID, GPIO.LOW)  
        GPIO.output(EN, GPIO.LOW)
        GPIO.output(DIR, GPIO.LOW)
        GPIO.output(PWM, GPIO.LOW)   
        trigger_winch.value = False   
        trigger_servo.value = False
        GPIO.cleanup()
