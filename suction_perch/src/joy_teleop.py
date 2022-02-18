#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Empty, Float64, Int8
from sensor_msgs.msg import Joy
import time

# define the default mapping between joystick buttons and their corresponding actions
# Logitech GamePad F310
ButtonSolenoid = 5 #for joypad RT
ButtonPump = 4 # for joypad LT
ButtonWinchUp = 3 # for joypad Y Yellow
ButtonWinchDown = 0 # for joypad A Green
ButtonWinchStop = 1 # for joypad B Red

rt_pressed = False
lt_pressed = False
winch_state = 0
 
def ReceiveJoystickMessage(data):
    global rt_pressed
    global lt_pressed
    global winch_state
    
    if data.buttons[ButtonPump]==1:
        if not lt_pressed:
            lt_pressed = True
    
    if data.buttons[ButtonPump]==0 and lt_pressed:
        rospy.loginfo("Pump Button Pressed and released")
        pub_pump.publish(Empty())
        lt_pressed = False

    if data.buttons[ButtonSolenoid]==1:
        if not rt_pressed:
            rt_pressed = True
    
    if data.buttons[ButtonSolenoid]==0 and rt_pressed:
        rospy.loginfo("Solenoid Button Pressed")
        pub_solenoid.publish(Empty())
        rt_pressed = False

    if data.buttons[ButtonWinchUp]==1:
        rospy.loginfo("Winch Up Button Pressed")
        winch_state = 1
        
    
    if data.buttons[ButtonWinchDown]==1:
        rospy.loginfo("Winch Down Button Pressed")
        winch_state = -1
        
    if data.buttons[ButtonWinchStop]==1:
        rospy.loginfo("Winch Stop Button Pressed")
        winch_state = 0
        
    pub_winch.publish(winch_state)

def main():
    global pub_pump
    global pub_solenoid
    global pub_winch

    rospy.init_node('joystick_teleop')
    rate = rospy.Rate(50) # 10hz

    # Next load in the parameters from the launch-file
    #ButtonRelay = int (   rospy.get_param("~ButtonRelay",ButtonRelay) )
    #ButtonPump      = int (   rospy.get_param("~ButtonPump",ButtonPump) )    

    subJoystick = rospy.Subscriber('joy', Joy, ReceiveJoystickMessage)
    pub_pump = rospy.Publisher('pump_on', Empty, queue_size=1)  
    pub_solenoid = rospy.Publisher('solenoid_on', Empty, queue_size=1)
    pub_winch = rospy.Publisher('winch_state', Int8, queue_size=1)
    

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
