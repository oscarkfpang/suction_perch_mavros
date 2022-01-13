#!/usr/bin/python

import rospy
import mavros
#from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, Float64, UInt16, Empty, Bool
#from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn, AttitudeTarget, Thrust, ParamValue
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamGet, ParamSet
#from mavros import command

#from multiprocessing import Value
#from ctypes import c_bool

from smbus import SMBus
import math
import time
import os

class ms4515_PI(object):
    STATUS_OK       = 0b00
    STATUS_RESERVED = 0b01
    STATUS_STALE    = 0b10
    STATUS_FAULT    = 0b11
    ADDR_READ_MR    = 0x00  # register address

    def __init__(self, i2c_bus=1, i2c_address=46, output_type='B', pressure_range=30, delay=0.1, ):
        '''
        i2c_bus => run the command: i2cdetect -l
        i2c_address => run the command: i2cdetect -y 1
        (1 is the i2c_bus returned later)
        '''

        # URL pointing where rosbridge_server is running
        self.output_type = output_type
        self.pressure_range = pressure_range
        self.delay = delay

        self.bus = SMBus(i2c_bus)
        self.address = int('0x'+str(i2c_address),0)

        self.dp_raw = 0
        self.dT_raw = 0
        self.status = 0
        self.diff_press_pa_raw = 0
        self.temp = 0

        self.rate = rospy.Rate(int(1.0/self.delay))
        self.pub_perch = rospy.Publisher('is_perched', Bool, queue_size=1)
        rospy.loginfo("ms4515 is initialised and ready")

    def setSuctionPerch(self, value):
        rospy.wait_for_service('/mavros/param/set')
        try:
            setSuctionPerch = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            param = ParamValue()
            param.integer = value
       
            suctionPerchChanged = setSuctionPerch("SUCTION_IS_PERCH", param)
            if suctionPerchChanged.success:
                return True
            else:
                return False
           
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s."%e )
            return False


    def measure(self):
        return self.bus.write_byte(self.address, 1)

    def collect(self):
        self.dp_raw = 0
        self.dT_raw = 0
        self.status = 0

        self.dp_raw = self.bus.read_byte(self.address)
        self.status = (self.dp_raw & 0xc0) >> 6   # get the first byte for status

        self.dp_raw = (self.dp_raw << 8) | self.bus.read_byte(self.address)
        self.dp_raw = 0x3fff & self.dp_raw

        self.dT_raw = self.bus.read_byte(self.address)
        self.dT_raw = (self.dT_raw << 8) | self.bus.read_byte(self.address)
        self.dT_raw = (0x3ffe0 & self.dT_raw) >> 5

        if (self.dp_raw == 0x3fff or self.dp_raw == 0 or self.dT_raw == 0x7ff or self.dT_raw == 0):
            return -1

        if (self.status == self.STATUS_STALE or self.status == self.STATUS_FAULT):
            return -1

        self.diff_press_pa_raw = self.get_pressure()
        self.temp = self.get_temperature()

    def get_pressure(self):
        P_max = self.pressure_range
        P_min = -P_max
        PSI_to_Pa = 6894.757

        if self.output_type == 'A':
            diff_press_PSI = -((self.dp_raw - 0.1 * 16383) * (P_max - P_min) / (0.8 * 16383) + P_min)
        else:    
            diff_press_PSI = -((self.dp_raw - 0.05 * 16383) * (P_max - P_min) / (0.9 * 16383) + P_min)
            
        return diff_press_PSI * PSI_to_Pa;

    def get_temperature(self):
        return ((200.0 * self.dT_raw) / 2047) - 50

    def run(self):
        rospy.loginfo("MS4515 has started!\n")
        try:
            while not rospy.is_shutdown():
                #start_time = time.time()

		ret = self.measure()
                #rospy.loginfo("ret = ", ret)
                self.collect()
                rospy.loginfo("pressure    = %s"%self.diff_press_pa_raw)
                #rospy.loginfo("temperature = %s", self.temp)
                #rospy.loginfo("=" * 50)
                if (self.diff_press_pa_raw > 2500):
                    if self.setSuctionPerch(True):
                        #rospy.loginfo("Set Perch param!")
                        self.pub_perch.publish(True)
                else:
                    if self.setSuctionPerch(False):
                        self.pub_perch.publish(False)
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("MS4515 has finished! \n")
        except:
            rospy.loginfo("We have a problem...")
            raise
        finally:
            self.bus.close()
            rospy.loginfo("Clean up smbus")


if __name__=="__main__":
    rospy.init_node("suction_perch_node")
    ms4515 = ms4515_PI()
    ms4515.run()
