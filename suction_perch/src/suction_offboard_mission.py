#!/usr/bin/env python
from __future__ import division

import rospy
import math
import numpy as np
import time
import sys
import signal
import argparse
from std_msgs.msg import Header, Empty, Bool, Float64
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, \
                            WaypointList, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush, CommandTOL
from sensor_msgs.msg import NavSatFix, Imu
#from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from threading import Thread
from tf.transformations import quaternion_from_euler
from multiprocessing import Value
from ctypes import c_float, c_int, c_bool
from collections import deque

class MavrosOffboardSuctionMission():
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def __init__(self, radius=0.1, vx=0.2, vy=0.2, vz=0.8, takeoff_alt=1.0,
                 mission_pos=((0, 0, 0, 0) , (0, 0, 5, 0), (5, 0, 2, 0), (2, 0, 3, 0), (0, 0, 3, 0)),
                 goto_pos_time=30, perch_time=60, land_on_wall_time=20, throttle_down_time=10):
        self.radius = radius # consider using a smaller radius in real flight
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.takeoff_alt = takeoff_alt
        self.goto_pos_time = goto_pos_time
        self.perch_time = perch_time
        self.land_on_wall_time = land_on_wall_time
        self.low_throttle_value = 0.5
        self.throttle_down_time = throttle_down_time
        self.throttle_down_start_time = -1   # > 0 for real time

        self.terminate = Value(c_bool, False)
        self.mission_cnt = Value(c_int, 0)
        self.mission_pos = mission_pos
        self.pump_on = Value(c_bool, False)
        self.solenoid_on = Value(c_bool, False)
        self.is_perched = Value(c_bool, False)
        self.publish_att_raw = Value(c_bool, False)   # ON: publish attitude_setpoint/raw for pitch up during vertical landing
        self.publish_thr_down = Value(c_bool, False)  # ON: toggle throttle down for vertical landing
        self.vtol = Value(c_bool, False)
        self.debug_mode = Value(c_bool, False)
        self.current_throttle = Value(c_float, 0.0)
        self.suction_pressure = 0.0

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/cmd/takeoff', service_timeout)
            rospy.wait_for_service('mavros/cmd/land', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")

        # mavros service
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.set_takeoff_srv = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.set_land_srv = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

        # mavros topics
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        #self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()

        self.pos = PoseStamped() # for setpoint_position
        self.pos_target = PositionTarget()

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'state', 'imu', 'local_pos', 'is_perched',
            ]
        }

        # ROS publishers
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pos_target_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.att_raw_setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.pub_pump = rospy.Publisher('pump_on', Empty, queue_size=1)
        self.pub_solenoid = rospy.Publisher('solenoid_on', Empty, queue_size=1)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
        #self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
        #                                      PoseStamped,
        #                                      self.local_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/vision_pose/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.perched_sub = rospy.Subscriber('is_perched',
                                              Bool,
                                              self.perched_callback)

        self.pressure_sub = rospy.Subscriber('suction_pressure',
                                               Float64,
                                               self.pressure_callback)
        # send mission pos setpoints in seperate thread to better prevent OFFBOARD failure
        # iterate list of pos setpoints
        self.pos_thread = Thread(target=self.send_mission_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True


    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True


    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    # safety callback for getting the suction status
    def perched_callback(self, data):
        self.is_perched.value = data
        if not self.sub_topics_ready['is_perched']:
            self.sub_topics_ready['is_perched'] = True
            
    def pressure_callback(self, data):
        self.suction_pressure = data
    #
    # Helper methods
    #

    # constantly publish waypoint (position / velocity) in a thread
    def send_mission_pos(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mission_cnt.value >= len(self.mission_pos):
                # handle exception when mission_cnt out of index range of mission_pos.
                self.pos_setpoint_pub.publish(self.make_origin_pos()) # publish a zero setpoint
            else:
                if self.mission_pos[self.mission_cnt.value][3] == 0: # normal pos setpoint
                    self.pos_setpoint_pub.publish(self.make_pos())
                elif self.mission_pos[self.mission_cnt.value][3] == 1: # velocity setpoint
                    if not self.publish_att_raw.value:
                        # publish velocity setpoint for slowly closing / retracting from the wall
                        self.pos_target_setpoint_pub.publish(self.make_pos_target()) 
                    else:
                        # publish attitude setpoint for high attitude movement
                        self.att_raw_setpoint_pub.publish(self.make_att_target())    
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
                
    def make_origin_pos(self):
        # set position setpoint to 1m above origin
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "mission_pos_origin"
        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 1
        pos.header.stamp = rospy.Time.now()
        return pos
        
    def make_pos(self):
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "mission_pos"
        pos.header.stamp = rospy.Time.now()
        #if self.mission_cnt.value < len(self.mission_pos):
        pos.pose.position.x = self.mission_pos[self.mission_cnt.value][0]
        pos.pose.position.y = self.mission_pos[self.mission_cnt.value][1]
        pos.pose.position.z = self.mission_pos[self.mission_cnt.value][2]
        return pos
        #else:
        #    return self.make_origin_pos()

    def make_pos_target(self):
        pos_target = PositionTarget()
        pos_target.header = Header()
        pos_target.header.frame_id = "mission_pos_target"
        pos_target.header.stamp = rospy.Time.now()
        pos_target.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                               PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_PX + \
                               PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
        pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pos_target.velocity.x = 0
        pos_target.velocity.y = 0
        pos_target.velocity.z = 0

        # send directional velocity command to the drone
        if self.mission_pos[self.mission_cnt.value][0] > 0:
            pos_target.velocity.x = 0.5
        elif self.mission_pos[self.mission_cnt.value][0] < 0:
            pos_target.velocity.x = -0.5
        if self.mission_pos[self.mission_cnt.value][1] > 0:
            pos_target.velocity.y = 0.5
        elif self.mission_pos[self.mission_cnt.value][1] < 0:
            pos_target.velocity.y = -0.5    
        if self.mission_pos[self.mission_cnt.value][2] > 0:
            pos_target.velocity.z = 0.5
        elif self.mission_pos[self.mission_cnt.value][2] < 0:
            pos_target.velocity.z = -0.5
            
        pos_target.yaw = 0 # always point to the front
        return pos_target

    def make_att_target(self):
        att_target = AttitudeTarget()
        att_target.header = Header()
        att_target.header.stamp = rospy.Time.now()
        # change these values by experiment
        roll = 0.0
        yaw = 0.0
        if not self.publish_thr_down.value:
            pitch = -0.2 # tested in jmavsim for -ve value = pitch up (flip backward)
            throttle = self.low_throttle_value
            att_target.header.frame_id = "high_pitch_low_throttle"
        else:
            # gradually throttling down
            pitch = -0.2 #0.0

            '''
            TODO: move the following calculation to main mission loop and publish the shared variable
                  of the throttle value for use in this thread, for reusing the code
            '''
            if self.throttle_down_start_time >= 0:
                att_target.header.frame_id = "throttling_to_zero"
                dt = rospy.get_time() - self.throttle_down_start_time
                if dt <= self.throttle_down_time:
                    throttle = (self.throttle_down_time - dt) / self.throttle_down_time * self.low_throttle_value
                else:
                    #self.throttle_down_start_time = -1
                    att_target.header.frame_id = "zero_throttle"
                    throttle = 0.0
            else:
                att_target.header.frame_id = "zero_throttle"
                throttle = 0.0

        att_target.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        vector3 = Vector3(x=roll , y=pitch , z=yaw)
        att_target.thrust = throttle
        att_target.body_rate = vector3
        return att_target



    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        if arm:
            rospy.loginfo("trying to ARM the drone")
        else:
            rospy.loginfo("trying to DISARM the drone")
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                if arm:
                    rospy.loginfo("Successfully ARM the drone in seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                else:
                    rospy.loginfo("Successfully DISARM the drone in seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                pass
                #self.fail(e)

        self.assertTrue(arm_set, (
            "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
            format(arm, old_arm, timeout)))
        return arm_set

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                pass
                #self.fail(e)

        self.assertTrue(mode_set, (
            "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout)))

    def takeoff(self, timeout=10):
        """mode: PX4 mode string, timeout(int): seconds"""
        #rospy.loginfo("setting FCU mode: {0}".format(mode))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        can_take_off = False
        for i in xrange(timeout * loop_freq):
            if not can_take_off:
                try:
                    #TODO: check return type of set_takeoff_srv
                    res = self.set_takeoff_srv(altitude = self.takeoff_alt)
                    rospy.loginfo("takeoff res = {0}".format(res))
                    can_take_off = True
                    rospy.loginfo("takeoff successful!")
                    break
                except rospy.ServiceException as e:
                    rospy.loginfo("takeoff call failed.. try again")
            try:
                rate.sleep()
            except rospy.ROSException as e:
                # TODO: add hold mode upon failure
                pass
        self.assertTrue(can_take_off, (
            "failed to take off in {0} sec".format(timeout)))
        return can_take_off

    def land(self, timeout=20):
        """mode: PX4 mode string, timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        can_land = False
        for i in xrange(timeout * loop_freq):
            if not can_land:
                try:
                    #TODO: check return type of set_land_srv
                    self.set_land_srv(altitude = self.takeoff_alt)
                    can_land = True
                    rospy.loginfo("land successful!")
                    break
                except rospy.ServiceException as e:
                    rospy.loginfo("land call failed.. try again")

            try:
                rate.sleep()
            except rospy.ROSException as e:
                # TODO: add hold mode upon failure
                pass
        self.assertTrue(can_land, (
            "failed to land in {0} sec".format(timeout)))
        return can_land

    def assertTrue(self, assertTrue, msg):
        if not assertTrue:
            rospy.logerr(msg)


    def is_at_position(self, offset):
        """offset: in meters"""
        desired = np.array((self.mission_pos[self.mission_cnt.value][0],
                            self.mission_pos[self.mission_cnt.value][1],
                            self.mission_pos[self.mission_cnt.value][2]))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        current_offset = np.linalg.norm(desired - pos)
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.3f}, y: {4:.3f}, z: {5:.3f} | offset:{6:.3f}".
            format(desired[0], desired[1], desired[2],
                   pos[0], pos[1], pos[2],
                   current_offset))
        return current_offset < offset

    def goto_position(self, timeout=30):
        """timeout(int): seconds"""
        # set a position setpoint
        x = self.mission_pos[self.mission_cnt.value][0]
        y = self.mission_pos[self.mission_cnt.value][1]
        z = self.mission_pos[self.mission_cnt.value][2]
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # turn off suction pump if it's on before moving to the next WP
        # TODO: add GPIO control here for better control of ON/OFF
        if self.pump_on.value:
            self.pub_pump.publish(Empty())
            self.pump_on.value = False
        # For demo purposes we will lock yaw/heading to north.
        #yaw_degrees = 0  # North
        #yaw = math.radians(yaw_degrees)
        #quaternion = quaternion_from_euler(0, 0, yaw)
        #self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail()

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))
        return reached

    def run_mission_perch(self):
        # make sure the simulation is ready to start the mission
        self.wait_for_mission_topics(60)
        self.mission_cnt.value = 0  # ground position
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        self.mission_cnt.value = 1
        mission_fail = False
        

        while self.mission_cnt.value < len(self.mission_pos):
            if self.mission_pos[self.mission_cnt.value][3] == 0: # normal pos setpoint
                if self.goto_position(self.goto_pos_time):
                    self.mission_cnt.value += 1
                else:
                    mission_fail = True
            elif self.mission_pos[self.mission_cnt.value][3] == 1: # velocity setpoint
                if self.perch_wall(self.perch_time):
                    self.mission_cnt.value += 1
                    # upon successful perching by suction cup, publish 0 vel setpoint for 3 sec for stabilisation
                    rospy.loginfo("publish 0 vel setpoint for 3 sec for stabilisation")
                    rospy.sleep(3)
                    # begin land on wall
                    if self.land_on_wall(self.land_on_wall_time, self.throttle_down_time):
                        self.vtol.value = True
                        rospy.loginfo("Vehicle attached to wall successfully!")
                        self.land()
                        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                                    10, -1)
                        rospy.loginfo("STATUS: Landed on vertical surface successfully! Disarm now.")
                        # disarm the drone
                        self.set_arm(False, 5)
                        break
                        # vechile landed successfully. break out the loop and save the status
                        # i.e. mission_fail = False
                        
                    else: # land fail
                        mission_fail = True
                        # drone fails to land to the wall. go to origin
                        rospy.loginfo("FAIL: Landing to wall unsuccessful! RTH now.")
                    ## move on to the next waypoint for testing
                    #self.mission_cnt.value += 1
                else: # perch fail
                    mission_fail = True 
                    # drone fails to perch to the wall. go to origin
                    rospy.loginfo("FAIL: Perching to wall unsuccessful! RTH now.")
                    
            #if self.vtol.value:
                #break
            if mission_fail:
                break
                    
        if mission_fail:
            # only turn off the suction if the drone is not pearched to the wall
            if not self.is_perched.value and self.pump_on.value:
                self.pub_pump.publish(Empty())
                self.pump_on.value = False
            rospy.loginfo("Drone RTH for emergency landing")
            self.mission_cnt.value = len(self.mission_pos)+1
            self.goto_position(self.goto_pos_time)
        
        if not mission_fail:
            self.mission_cnt.value += 1
            rospy.loginfo("STATUS: current mission_cnt value = {0}".format(self.mission_cnt.value))
            
        ## TODO: check if it mission is successful and return True directly
        
        '''
        self.land()
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                     10, -1)
                                     
                                    
        self.set_arm(False, 5)
        rospy.loginfo("Drone landed and disarm")
        '''
        
        
        return True

    def run_mission_full(self):
        rospy.loginfo("Full Mission Start...")
        
        perch_successful = False
        retakeoff_successful = False
        detach_successful = False
        perch_successful = self.run_mission_perch()
        
        if not perch_successful:
            rospy.loginfo("STAUTS: Perching is not successful! Stop here!")
            return False
            
        rospy.loginfo("STATUS: Sleep for 10 sec before re-takeoff")        
        rospy.sleep(10)
        rospy.loginfo("STATUS: Perching is successful. Now re-takeoff!")
        
        retakeoff_successful = self.takeoff_from_wall()
        
        if not retakeoff_successful:
            rospy.loginfo("STAUTS: Re-takeoff is not successful! Stop here!")
            return False

        rospy.loginfo("STATUS: Re-takeoff is successful. Now detach from wall!")
        detach_successful = self.detach_from_wall()
        
        if not detach_successful:
            rospy.loginfo("STAUTS: Detach is not successful! Stop here!")
            return False
            
        rospy.loginfo("STATUS: Detach is successful. Fly back!")      
        self.throttle_down_start_time = -1
        
        # suppose self.mission_cnt.value = 5 already
        rospy.loginfo("STATUS: Go to waypoint 5")
        if self.goto_position(self.goto_pos_time): # for WP 5
            self.mission_cnt.value += 1
        
        rospy.loginfo("STATUS: Go to waypoint 6 (1, 0, 1.5, 0)")
        if self.goto_position(self.goto_pos_time): # for WP 6
            self.mission_cnt.value += 1           
        
        rospy.loginfo("STATUS: Go to waypoint 7 (0.5, 0, 0.5, 0)")
        self.goto_position(self.goto_pos_time) # for WP 7
                    

        self.land()
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                     10, -1)
        rospy.loginfo("Disarm for finishing mission!!")
        self.set_arm(False, 5)         
        
        rospy.loginfo("=================== Mission Successful! =========================+")
        return True

    '''
    def run_mission_rearm(self):
        rospy.loginfo("Rearm the drone from vertical pose.")
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        self.publish_thr_down.value = False
        rospy.loginfo("Throttle set to 0.5")
        rospy.loginfo("Keep for 20 sec and set throttle to zero again.")
        rospy.sleep(20)
        self.publish_thr_down.value = True
        self.throttle_down_start_time = rospy.get_time()
        rospy.sleep(20)
        rospy.loginfo("Disarm again")
        self.set_arm(False, 5)        
        rospy.loginfo("End rearm mission")  
    '''      

    def run_mission_by_hand(self):
        rospy.loginfo("Hand-fly Mission Start...")
        self.debug_mode.value = True
        
        perch_successful = False
        retakeoff_successful = False
        detach_successful = False
        perch_successful = self.run_mission_perch()
        
        if not perch_successful:
            rospy.loginfo("STAUTS: Perching is not successful! Stop here!")
            return False
            
        rospy.loginfo("STATUS: Sleep for 10 sec before re-takeoff")        
        rospy.sleep(10)
        rospy.loginfo("STATUS: Perching is successful. Now re-takeoff!")
        
        retakeoff_successful = self.takeoff_from_wall()
        
        if not retakeoff_successful:
            rospy.loginfo("STAUTS: Re-takeoff is not successful! Stop here!")
            return False

        rospy.loginfo("STATUS: Re-takeoff is successful. Now detach from wall!")
        detach_successful = self.detach_from_wall()
        
        if not detach_successful:
            rospy.loginfo("STAUTS: Detach is not successful! Stop here!")
            return False
            
        rospy.loginfo("STATUS: Detach is successful. Finish Flying by hand!")      
        
        rospy.sleep(10)
        self.throttle_down_start_time = -1
        self.land()
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                     10, -1)
        rospy.loginfo("Disarm for finishing mission")
        self.set_arm(False, 5)         
        return True
        

    def run_mission_square(self):
        # make sure the simulation is ready to start the mission
        self.wait_for_mission_topics(60)
        #self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        #                           10, -1)
        self.mission_cnt.value = 0  # ground position
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        self.mission_cnt.value = 1
        # doesn't work in sim for takeoff service call
        #if self.takeoff():
        #    self.mission_cnt.value = 1  # take off position
        #else:
        #    rospy.logerr("Mission Failed!")
        #    return -1
        #rospy.sleep(30)

        while self.mission_cnt.value < len(self.mission_pos):
            if self.goto_position(30):
                self.mission_cnt.value += 1
            else:
                break

        self.land()
        #rospy.loginfo("Mission completed... wait for 3 sec before landing.")
        #rospy.sleep(3)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)
        self.set_arm(False, 5)
        self.set_mode("STABILIZED", 5)
        return       

    def is_high_attitude(self):
        rospy.loginfo("IMU data.y = {0}".format(self.imu_data.orientation.y))
        return self.imu_data.orientation.y > 0.30 and self.imu_data.orientation.y < 0.7
        
    def is_normal_attitude(self):
        rospy.loginfo("IMU data.y = {0}".format(self.imu_data.orientation.y))
        return self.imu_data.orientation.y < 0.1

    def is_vertical_takeoff_attitude(self):
        rospy.loginfo("IMU data.y = {0}".format(self.imu_data.orientation.y))
        return self.imu_data.orientation.y < 0.52

    def auto_throttling(self, start_throttle, end_throttle, start_time, period):
        '''
        Adjust and publish throttle value for gradual throttle up / down
        start_throttle: initial throttle value
        end_throttle: end throttle value
        start_time: time to start throttling
        period: period of throttling
        Return float type of throttle value
        '''
        dt = rospy.get_time() - start_time
        while dt <= period:
            rospy.loginfo("MSG: Auto_throttling. current throttle = {0}".format(self.current_throttle.value))
            try:
                if end_throttle < start_throttle: # throttle down
                    self.current_throttle.value = (period - dt) / period * (start_throttle - end_throttle) + end_throttle
                else: # throttle up
                    self.current_throttle.value = end_throttle - (period - dt) / period * (end_throttle - start_throttle)             
                rospy.sleep(0.2)
                dt = rospy.get_time() - start_time
            except rospy.ROSException as e:
                # TODO: handling of throttle value
                self.current_throttle.value = start_throttle
                rospy.logfatal("STATUS: Auto_throttling is interrupted!")
                break
                

    def takeoff_from_wall(self, timeout=60, throttle_timeout=15):
        rospy.loginfo("STATUS: Set OFFBOARD mode.")
        self.set_mode("OFFBOARD", 5)
        rospy.loginfo("STATUS: Rearm the drone in vertical pose.")
        self.set_arm(True, 5)
        self.publish_att_raw.value = True
        self.publish_thr_down.value = False
        rospy.loginfo("STATUS: Throttle set from 0.2 to 0.5")

        #self.auto_throttling(start_throttle = 0.0, 
        #                     end_throttle = self.low_throttle_value, 
        #                     start_time = rospy.get_time(), 
        #                     period = 5)

        start_throttle = 0.2
        end_throttle = 0.5 # self.low_throttle_value 
        self.throttle_up_start_time = rospy.get_time()

        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        period = throttle_timeout * loop_freq
        takeoff_from_vertical = False

        for i in xrange(throttle_timeout * loop_freq):
            rospy.loginfo("MSG: Auto_throttling up from 0.2. current throttle = {0}".format(self.current_throttle.value))
            try:
                # throttling up
                self.current_throttle.value = start_throttle + i / period * (end_throttle - start_throttle)             

                # detect SUCTION_IS_LAND param while throttling up
                res = self.get_param_srv('SUCTION_IS_LAND')
                if res.success and res.value.integer <= 0:
                    rospy.loginfo(
                        "SUCTION_IS_LAND received {0}. drone takes off vertically from the wall! ".format(res.value.integer))
                    takeoff_from_vertical = True
                    break
                    # no need break out the loop here. Wait for the whole period

                rate.sleep()
            except rospy.ROSException as e:
                # TODO: handling of throttle value under failure
                self.current_throttle.value = 0.0
                rospy.logfatal("STATUS: Auto_throttling is interrupted!")
                break

        if not takeoff_from_vertical:
            self.assertTrue(takeoff_from_vertical, (
                "took too long to take off from wall | timeout(seconds): {0}".format(timeout)))
            self.publish_att_raw.value = True
            return False

        start_throttle = self.current_throttle.value
        end_throttle = 0.7
        period = timeout * loop_freq
        normal_attitude = False
        rospy.loginfo("Gradually throttling up to level attitude.")
        for i in xrange(timeout * loop_freq):
            try:                    
                if self.is_normal_attitude():
                    normal_attitude = True
                    break
                self.current_throttle.value = start_throttle + i / period * (end_throttle - start_throttle)
                rate.sleep()
            except rospy.ROSException as e:
                pass

        # TODO: assert self.current_throttle < 0.7
        if not normal_attitude:
            pass

        return normal_attitude


    #TODO: publish attitude_raw setpoint for perching and landing on the wall
    def land_on_wall(self, timeout=10, throttle_timeout=5):
        rospy.loginfo(
             "========= waiting for SUCTION_IS_LAND =========")
        rospy.loginfo(
             "High Attitude Movement is about to start... ")
        self.publish_att_raw.value = True
        self.publish_thr_down.value = False # keep throttle > 0 throughout high-attitude movement

        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        vertical_landing = False
        pitch_up = False
        
        #if self.debug_mode.value:
        #    pitch_up = True

        for i in xrange(timeout * loop_freq, 0, -1):
            rospy.loginfo(
                        "** High Attitude Movement. Count-down to end: {0}".format(i))
            try:                    
                if self.is_high_attitude():
                    pitch_up = True
                    break
                rate.sleep()
            except rospy.ROSException as e:
                pass

        if not pitch_up:
            self.assertTrue(pitch_up, (
                "took too long to reach high attitude | timeout(seconds): {0}".format(timeout)))
            self.publish_att_raw.value = False
            return False

        # throttle down at high pitch angle vertical landing phase
        rospy.loginfo(
            "Throttle down from high attitude is about to start... ")
        self.publish_thr_down.value = True
        self.throttle_down_start_time = rospy.get_time()
        # gradually reduce throttle to zero during throttle_timeout
        for i in xrange(throttle_timeout * loop_freq):
            try:
                # detect SUCTION_IS_LAND param while throttling down
                res = self.get_param_srv('SUCTION_IS_LAND')
                if res.success and res.value.integer > 0:
                    rospy.loginfo(
                        "SUCTION_IS_LAND received {0}. drone is landed vertically to the wall! ".format(res.value.integer))
                    vertical_landing = True
                    # no need break out the loop here. Wait for the whole throttle_timeout period
                    #break
                
                # detect if is_perched is False
                if self.debug_mode.value and not self.is_perched.value:
                    rospy.loginfo("Perching is interrupted! Halt the throttle down process!")
                    break
                rate.sleep()
                rospy.loginfo("Set Throttle = 0 during vetical landing")
            except rospy.ROSException as e:
                pass

        # turn off att_raw_setpoint publishing
        ## self.publish_att_raw.value = False      # no need as we keep publishing 0 att_raw

        if not vertical_landing:
            self.assertTrue(vertical_landing, (
                "took too long to reach high attitude | timeout(seconds): {0}".format(timeout)))
            self.publish_att_raw.value = False
            self.publish_thr_down.value = False

        return vertical_landing
    
    def detach_from_wall(self, timeout=20):
        '''
        Detach the drone from the wall. Precondition: the drone returns to normal attitude after re-takeoff
        return successful detachment or not
        '''
        
        rospy.loginfo("Begin detaching from wall")
        
        '''
        # assert suction pressure is still good and solenoid is on
        if not self.solenoid_on.value:
            rospy.logfatal("Solenoid is off! Airflow is not normal! Please check!")
            return False
        if not self.is_perched:
            rospy.logfatal("Suction pressure is lost or the drone is not perched! Please check!")
            return False
        '''


        # turn off solenoid and fly away from wall
        if self.solenoid_on.value:
            rospy.loginfo("Turn off solenoid")
            self.pub_solenoid.publish(Empty())
            self.solenoid_on.value = False
            

        # check suction pressure in a loop until suction cup is detached from the wall
        # does it perch to the wall in 'timeout' seconds?
        rospy.loginfo("========= waiting for SUCTION_IS_PERCH for detach =========")
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        detach = False
        for i in xrange(timeout * loop_freq, 0, -1):
            rospy.loginfo(
                        "waiting for SUCTION_IS_PERCH to 0. Current Pressure = {0} | Time left {1} sec".format(self.suction_pressure, i))
            try:
                res = self.get_param_srv('SUCTION_IS_PERCH')
                if res.success and res.value.integer <= 0:
                    rospy.loginfo(
                        "SUCTION_IS_PERCH received {0}. drone detaches from the wall! ".format(res.value.integer))
                    detach = True
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
                
        if detach:
            self.publish_att_raw.value = False
            self.mission_cnt.value = 6

            if self.pump_on.value:
                rospy.loginfo("Turn off suction pump")
                self.pub_pump.publish(Empty())
                self.pump_on.value = False
            
        return detach

    def perch_wall(self, timeout=60):
        # turn on suction motor
        if not self.pump_on.value:
            rospy.loginfo("Turn on suction pump")
            self.pub_pump.publish(Empty())
            self.pump_on.value = True
            
        # turn on solenoid (default is off)
        if not self.solenoid_on.value:
            rospy.loginfo("Turn on Solenoid")
            self.pub_solenoid.publish(Empty())
            self.solenoid_on.value = True

        # check suction pressure in a loop until suction cup is attaced to the wall
        # does it perch to the wall in 'timeout' seconds?
        rospy.loginfo("========= waiting for SUCTION_IS_PERCH for perching =========")
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        suction = False
        for i in xrange(timeout * loop_freq, 0, -1):
            rospy.loginfo(
                        "waiting for SUCTION_IS_PERCH to 1. Current Pressure = {0} | Time left {1} sec".format(self.suction_pressure, i))
            # TODO: give a short period of time for detecting the continuous reception of value 1
            try:
                res = self.get_param_srv('SUCTION_IS_PERCH')
                if res.success and res.value.integer > 0:
                    ##self.suction_is_perch_param = res.integer
                    rospy.loginfo(
                        "SUCTION_IS_PERCH received {0}. drone is perched to the wall! ".format(res.value.integer))
                    suction = True
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
        
        #TODO: assert suction_is_perch param = is_pearch ROS topic
        #
        #

        if not suction:
            # turn off suction pump if fail
            if self.pump_on.value:
                self.pub_pump.publish(Empty())
                self.pump_on.value = False
        self.assertTrue(suction, (
            "took too long to get suction pressure | timeout(seconds): {0}".format(timeout)))
        return suction


    def fail(self, e=None):
        if e is not None:
            if type(e) == str:
                rospy.loginfo(e)
            else:
                rospy.loginfo(str(e))
        self.terminate.value = True
        # terminate the thread if exists
        if hasattr(self, 'pos_thread') and self.pos_thread is not None:
            rospy.loginfo("Setpoint Publishing Thread terminates!")
            self.pos_thread.join()
        # turn off the pump
        if self.pump_on.value:
            rospy.loginfo("Turn OFF suction pump")
            self.pub_pump.publish(Empty())
            self.pump_on.value = False

        rospy.loginfo("======= STOP OFFBOARD & EXIT======= ")
        sys.exit(0)

    #
    # Test method
    #
    def test(self):
        rate = rospy.Rate(1)
        self.mission_cnt.value = 0

        while self.mission_cnt.value < len(self.mission_pos):
            rospy.loginfo("mission_cnt = {0}".format(self.mission_cnt.value))
            rate.sleep()
            self.mission_cnt.value += 1
            if self.mission_cnt.value >= len(self.mission_pos):
                self.mission_cnt.value = 0




    # define a new method similar to the original wait_for_mission for real-life operation
    # wait until all mavros topics to be ready
    def wait_for_mission_topics(self, timeout):
        """wait for mavros topics to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        operation_ready = False
        rospy.loginfo("All topics?= {0}".format(self.sub_topics_ready['alt']))
        rospy.loginfo("All topics?= {0}".format(self.sub_topics_ready['ext_state']))
        rospy.loginfo("All topics?= {0}".format(self.sub_topics_ready['imu']))
        rospy.loginfo("All topics?= {0}".format(self.sub_topics_ready['state']))
        for i in xrange(timeout * loop_freq):
            topics = [self.sub_topics_ready['alt'], self.sub_topics_ready['ext_state'], self.sub_topics_ready['imu'],
                    self.sub_topics_ready['state']]
            rospy.loginfo("All topics?= {0}".format(topics))
            if all(topics):
                operation_ready = True
                rospy.loginfo("Mavros topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(operation_ready, (
            "failed to hear from all subscribed mavros topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready, timeout)))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(landed_state_confirmed, (
            "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                desired_landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                   index, timeout)))

# to shut down the ROS Service gracefully after pressing Ctrl-C
def sigint_handler(signum, data):
    rospy.loginfo('Ctrl-C is pressed.')
    rospy.signal_shutdown('Wait for 5 sec and shutting down ROS node...')
    rospy.sleep(5)
    sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Mission Script for Suction Perch Drone")
    parser.add_argument('-d', '--debug', action='store_true', help="debug output")
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('-sq', '--sq-test', action='store_true', help="fly a square waypoint")
    mode_group.add_argument('-vel', '--vel-test', action='store_true', help="fly forward and change to velocity setpoint")
    mode_group.add_argument('-hand', '--hand-test', action='store_true', help="move the drone around using hand")
    mode_group.add_argument('-rearm', '--rearm-test', action='store_true', help="rearm the drone after landing to the wall")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('suction_mission_node')
    #signal.signal(signal.SIGINT, sigint_handler)
    #signal.signal(signal.SIGTERM, sigint_handler)

    # mission waypoints for perching test
    mission_pos_vel = ((0, 0, 0, 0) , (0, 0.1, 1.5, 0), (1, 0.1, 1.5, 0), (1, 0, 0, 1), (0, 0, 0, 1), (1, 0.1, 1.5, 0), (0, 0, 1.5, 0))
    # mission waypoints for flying a square box
    mission_pos_sq = ((0, 0, 0, 0) , (0, 0, 1.5, 0), (-1, -1, 1.5, 0), (-1, 1, 1.5, 0), (1, 1, 1.5, 0), (1, -1, 1.5, 0), (0, 0, 1.5, 0)) 
    # mission waypoints for velocity setpoint test
    mission_pos_vel_test = ((0, 0, 0, 0) , (0, 0, 2, 0), (1, 0, 2, 0), (0, 1, 0, 1), (0, 0, 0, 1), (1, 0, 2, 0), (0, 0, 2, 0))
    
    # mission waypoints for perching test
    mission_pos_hand = ((0, 0, 0, 0) , (0, 0, 1.5, 0), (1, 0, 1.5, 0), 
                        (1, 0, 0, 1), (0, 0, 0, 1), (-1, 0, 0, 1),
                        (1, 0, 1.5, 0), (0.5, 0, 0.5, 0), (0, 0, 0, 0))

    global suction_mission

    if args.sq_test:
        suction_mission = MavrosOffboardSuctionMission(mission_pos=mission_pos_sq)
        suction_mission.run_mission_square()
    elif args.vel_test:
        suction_mission = MavrosOffboardSuctionMission(radius=0.1,
                                                       mission_pos=mission_pos_vel,
                                                       goto_pos_time=60, perch_time=30, land_on_wall_time=30, throttle_down_time=10)
        suction_mission.run_mission_perch()
    elif args.rearm_test:
        suction_mission = MavrosOffboardSuctionMission(radius=0.1,
                                                       mission_pos=mission_pos_hand,
                                                       goto_pos_time=60, perch_time=50, land_on_wall_time=40, throttle_down_time=10)
        suction_mission.run_mission_full()
    elif args.hand_test:
        suction_mission = MavrosOffboardSuctionMission(radius=0.4,
                                                       mission_pos=mission_pos_hand,
                                                       goto_pos_time=60, perch_time=30, land_on_wall_time=30, throttle_down_time=10)
        suction_mission.run_mission_by_hand()
        

    rospy.spin()
    '''
    try:
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo("running... ")
    except rospy.ROSInterruptException:
        rospy.loginfo("main has finished! \n")

    '''

