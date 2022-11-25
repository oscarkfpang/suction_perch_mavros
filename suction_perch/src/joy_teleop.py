#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty, Float64, Int8, Bool
from sensor_msgs.msg import Joy
import time

# define the default mapping between joystick buttons and their corresponding actions
# Logitech GamePad F310
ButtonSolenoid = 5 #for joypad RT
ButtonPump = 4 # for joypad LT
ButtonWinchUp = 3 # for joypad Y Yellow
ButtonWinchDown = 0 # for joypad A Green
ButtonWinchStop = 1 # for joypad B Red
ButtonServo = 2 # for joypad X Blue
ButtonSmallPump = 6 # for joypad BACK button
ButtonOverride = 7 # for start button
ButtonLogitech = 8 # for logitech button

rt_pressed = False
lt_pressed = False
back_pressed = False
winch_state = 0
x_pressed = False
servo_state = False

s_pressed = False
override_state = True

winch_done = False
logitech_pressed = False
 
def ReceiveJoystickMessage(data):
    global rt_pressed
    global lt_pressed
    global back_pressed
    global winch_state
    global x_pressed
    global servo_state
    global s_pressed
    global override_state
    global logitech_pressed
    global winch_done

    if data.buttons[ButtonOverride]==1:
        if not s_pressed:
            s_pressed = True
    
    if data.buttons[ButtonOverride]==0 and s_pressed:
        rospy.loginfo("Override Button pressed and released")
        #s_pressed = False
        if override_state:
            override_state = False
        else:
            override_state = True
    
    if data.buttons[ButtonPump]==1:
        if not lt_pressed:
            lt_pressed = True
    
    if data.buttons[ButtonPump]==0 and lt_pressed:
        rospy.loginfo("Pump Button Pressed and released")
        pub_pump.publish(Empty())
        lt_pressed = False

    
    if data.buttons[ButtonSmallPump]==1:
        if not back_pressed:
            back_pressed = True
    
    if data.buttons[ButtonSmallPump]==0 and back_pressed:
        rospy.loginfo("Small Pump Button Pressed and released")
        pub_small_pump.publish(Empty())
        back_pressed = False


    if data.buttons[ButtonSolenoid]==1:
        if not rt_pressed:
            rt_pressed = True
    
    if data.buttons[ButtonSolenoid]==0 and rt_pressed:
        rospy.loginfo("Solenoid Button Pressed")
        pub_solenoid.publish(Empty())
        rt_pressed = False

    if data.buttons[ButtonWinchUp]==1:
        rospy.loginfo("Winch Up Button Pressed")
        winch_state = 1.0        
        pub_winch_up.publish(Empty())
    
    if data.buttons[ButtonWinchDown]==1:
        rospy.loginfo("Winch Down Button Pressed")
        winch_state = -1.0
        pub_winch_down.publish(Empty())
        
    if data.buttons[ButtonWinchStop]==1:
        rospy.loginfo("Winch Stop Button Pressed")
        winch_state = 0.0
        pub_winch_stop.publish(Empty())
    
    if data.buttons[ButtonServo]==1:
        if not x_pressed:
            x_pressed = True
    
    if data.buttons[ButtonServo]==0 and x_pressed:
        if servo_state:
            servo_state = False
        else:
            servo_state = True
        rospy.loginfo("Servo state = {0}".format(servo_state))
        x_pressed = False

    if data.buttons[ButtonLogitech]==1:
        if not logitech_pressed:
            logitech_pressed = True
    
    if data.buttons[ButtonLogitech]==0 and logitech_pressed:
        rospy.loginfo("Logitech Button Pressed and released")
        if winch_done:
            winch_done = False
        else:
            winch_done = True
        logitech_pressed = False

def main():
    global pub_pump
    global pub_solenoid
    global pub_small_pump
    global pub_winch_stop
    global pub_winch_up
    global pub_winch_down
    
 
    rospy.init_node('joystick_teleop')
    rate = rospy.Rate(50) # 10hz

    # Next load in the parameters from the launch-file
    #ButtonRelay = int (   rospy.get_param("~ButtonRelay",ButtonRelay) )
    #ButtonPump      = int (   rospy.get_param("~ButtonPump",ButtonPump) )    

    subJoystick = rospy.Subscriber('joy', Joy, ReceiveJoystickMessage)
    pub_pump = rospy.Publisher('pump_on', Empty, queue_size=1)  
    pub_solenoid = rospy.Publisher('solenoid_on', Empty, queue_size=1)
    pub_small_pump = rospy.Publisher('small_pump_on', Empty, queue_size=1)  
    pub_winch = rospy.Publisher('winch_state', Float64, queue_size=1)
    pub_winch_up = rospy.Publisher('winch_cmd_up', Empty, queue_size=1)
    pub_winch_down = rospy.Publisher('winch_cmd_down', Empty, queue_size=1)
    pub_winch_stop = rospy.Publisher('winch_cmd_stop', Empty, queue_size=1)    
    pub_servo = rospy.Publisher('servo_state', Bool, queue_size=1)
    pub_override = rospy.Publisher('winch_override', Bool, queue_size=1)
    pub_winch_done = rospy.Publisher('winch_done', Bool, queue_size=1)
    
    try:
        while not rospy.is_shutdown():
            pub_override.publish(override_state)    
            pub_winch.publish(winch_state)
            pub_servo.publish(servo_state)
            pub_winch_done.publish(winch_done)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joystick Teleop stops! Force Landing")
    finally:
        rospy.loginfo("Node stops. Bye-bye!")


if __name__ == '__main__':
    main()
