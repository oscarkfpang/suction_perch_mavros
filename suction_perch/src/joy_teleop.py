#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String, Empty, Float64

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

#import RPi.GPIO as GPIO           
import time

# define the default mapping between joystick buttons and their corresponding actions
ButtonSolenoid = 5 #for joypad RT
ButtonPump = 4 # for joypad Green A
#RelayPin = 18 # GPIO18

#GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD  
#GPIO.setup(18, GPIO.OUT)           # set GPIO18 as an output   

def ReceiveJoystickMessage(data):
    if data.buttons[ButtonPump]==1:
        rospy.loginfo("Pump Button Pressed")
        pub_pump.publish(Empty())

    #pub_relay.publish(data.buttons[ButtonSolenoid])

    if data.buttons[ButtonSolenoid]==1:
        rospy.loginfo("Solenoid Button Pressed")
        pub_solenoid.publish(Empty())




def main():
    global pub_pump
    global pub_solenoid

    rospy.init_node('joystick_teleop')
    rate = rospy.Rate(50) # 10hz

    
    # Next load in the parameters from the launch-file
    #ButtonRelay = int (   rospy.get_param("~ButtonRelay",ButtonRelay) )
    #ButtonPump      = int (   rospy.get_param("~ButtonPump",ButtonPump) )    

    subJoystick = rospy.Subscriber('joy', Joy, ReceiveJoystickMessage)
    pub_pump = rospy.Publisher('pump_on', Empty, queue_size=1)  
    pub_solenoid = rospy.Publisher('solenoid_on', Empty, queue_size=1)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joystick Teleop stops! Force Landing")
        pub_landing.publish(Empty())
    finally:
        rospy.loginfo("Node stops. Bye-bye!")


if __name__ == '__main__':
    main()
