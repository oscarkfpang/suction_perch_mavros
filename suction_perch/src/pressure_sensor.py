#!/usr/bin/python

import rospy
import mavros
from std_msgs.msg import Bool, Float64
from mavros_msgs.msg import OverrideRCIn, AttitudeTarget, Thrust, ParamValue
from mavros_msgs.srv import ParamGet, ParamSet

from smbus import SMBus
import math
import time
import os
import numpy as np

class ms4515_PI(object):
    # register address
    STATUS_OK       = 0b00
    STATUS_RESERVED = 0b01
    STATUS_STALE    = 0b10
    STATUS_FAULT    = 0b11
    ADDR_READ_MR    = 0x00  
    
    VACCUM_PRESSURE = -200000
    OPEN_PRESSURE   = -130000
    TRANSITION_PRESSURE = 20000

    def __init__(self, i2c_bus=1, i2c_address=46, output_type='B', pressure_range=30, delay=0.1, sample_time=6, debug=False ):
        '''
        i2c_bus => run the command: i2cdetect -l
        i2c_address => run the command: i2cdetect -y 1
        (1 is the i2c_bus returned later)
        '''

        self.output_type = output_type
        self.pressure_range = pressure_range
        self.delay = delay
        self.debug = debug

        self.bus = SMBus(i2c_bus)
        self.address = int('0x'+str(i2c_address),0)

        self.dp_raw = 0
        self.dT_raw = 0
        self.status = 0
        self.diff_press_pa_raw = 0
        self.temp = 0

        self.rate = rospy.Rate(int(1.0/self.delay))
        self.sample_time = sample_time
        self.queue_size = int(self.sample_time / self.delay)
        self.pressure_queue = np.zeros(self.queue_size)
        
        self.pub_perch = rospy.Publisher('is_perched', Bool, queue_size=1)
        self.pub_pressure = rospy.Publisher('suction_pressure', Float64, queue_size=1)
        rospy.loginfo("ms4515 pressure sensor is initialised and ready")

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

    def is_continuous_suction(self, new_pressure):
        # pop left and append right
        self.pressure_queue[0:-1] = self.pressure_queue[1:]
        self.pressure_queue[-1] = new_pressure
        return np.all(self.pressure_queue < self.VACCUM_PRESSURE)
        
        return continuous

    def run(self):
        rospy.loginfo("Sampling air pressure")
        
        # reset px4 suction parameter to 0
        self.setSuctionPerch(False)
        
        try:
            while not rospy.is_shutdown():
                #start_time = time.time()
                ret = self.measure()
                self.collect()
                if self.debug:
                    rospy.loginfo("pressure    = %s", self.diff_press_pa_raw)
                    #rospy.loginfo("temperature = %s", self.temp)
                self.pub_pressure.publish(self.diff_press_pa_raw)
                #if self.is_continuous_suction(self.diff_press_pa_raw):
                #    self.pub_perch.publish(True)
                #else:
                #    self.pub_perch.publish(False)
                    
                '''
                if (self.diff_press_pa_raw < self.VACCUM_PRESSURE):
                    if self.setSuctionPerch(True):
                        self.pub_perch.publish(True)
                    else:
                        self.pub_perch.publish(False)
                elif (self.VACCUM_PRESSURE <= self.diff_press_pa_raw < self.OPEN_PRESSURE):
                    # TODO: transition phase... can do nothing here
                    pass
                elif (self.diff_press_pa_raw >= self.OPEN_PRESSURE):
                    if self.setSuctionPerch(False):
                        self.pub_perch.publish(False)
                '''
                
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("MS4515 has finished!")
        except:
            rospy.loginfo("We have a problem...")
            raise
        finally:
            self.bus.close()
            rospy.loginfo("Clean up smbus")


if __name__=="__main__":
    rospy.init_node("Airpressure_Node")
    ms4515 = ms4515_PI(debug = True)
    ms4515.run()
