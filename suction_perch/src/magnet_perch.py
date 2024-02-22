#!/usr/bin/env python3
from __future__ import division

import rospy
import math
import numpy as np
import sys
import argparse
from std_msgs.msg import Header, Empty, Bool, Float64
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, \
                            WaypointList, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush, CommandTOL
from sensor_msgs.msg import Imu
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

    def __init__(self, radius=0.1, vx=0.1, vy=0.1, vz=0.8, takeoff_alt=1.0,
                 mission_pos=((0, 0, 0, 0) , (0, 0, 5, 0), (5, 0, 2, 0), (2, 0, 3, 0), (0, 0, 3, 0)),
                 goto_pos_time=30, perch_time=60, land_on_wall_time=20, throttle_down_time=10):
        self.radius = radius # consider using a smaller radius in real flight
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.pitch_rate = -0.3
        self.holding_throttle = 0.5 ## 0.7 # 0.85 - OP 24-5-22
        self.takeoff_alt = takeoff_alt
        self.goto_pos_time = goto_pos_time
        self.perch_time = perch_time
        self.land_on_wall_time = land_on_wall_time
        self.low_throttle_value = 0.5 # 0.7 for sensor box paylod. 0.5 for no payload
        self.throttle_down_time = throttle_down_time
        self.throttle_down_start_time = -1   # > 0 for real time
        self.min_distance_from_wall = 0.15

        self.terminate = Value(c_bool, False)
        self.mission_cnt = Value(c_int, 0)
        self.mission_pos = mission_pos
        self.pump_on = Value(c_bool, False)
        self.solenoid_on = Value(c_bool, False)
        self.is_perched = Value(c_bool, False)
        self.publish_att_raw = Value(c_bool, False)   # ON: publish attitude_setpoint/raw for pitch up during vertical landing
        self.publish_thr_down = Value(c_bool, False)  # ON: toggle throttle down for vertical landing
        self.publish_thr_up = Value(c_bool, False)
        self.vtol = Value(c_bool, False)
        self.debug_mode = Value(c_bool, False)
        self.current_throttle = Value(c_float, 0.0)
        self.suction_pressure = 0.0
        self.winch_done = Value(c_bool, False)
        self.tfmini_range = Value(c_float, 0.0)
        

        ## for magnetic perching test ##
        self.publish_zero_sp_vel = Value(c_bool, False)
        self.publish_landing_sp_raw = Value(c_bool, False)
        self.target_pitch_rate = Value(c_float, 0.0)

        self.stationary = Value(c_bool, False)
        self.pull_off = Value(c_bool, False)
        self.APPROACH = 0
        self.STATIONARY_HORIZONTAL = 1
        self.PITCH_TO_VERTICAL = 2
        self.LAND_VERTICAL = 4
        self.TAKE_OFF_VERTICAL = 5
        self.PITCH_TO_HORIZONTAL = 6
        self.DETACH = 7
        self.FAIL = -1
        self.current_state = Value(c_int, self.STATIONARY_HORIZONTAL)

        self.user_interrupted = Value(c_bool, False)

        ## ========================== ##       


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
        self.winch_done_sub = rospy.Subscriber('winch_done',
                                               Bool,
                                               self.winch_state_callback)
        self.tfmini_sub = rospy.Subscriber('/mavros/distance_sensor/tfmini_pub',
                                               Float64,
                                               self.tfmini_callback)
        
                                               
        # send mission pos setpoints in seperate thread to better prevent OFFBOARD failure
        # iterate list of pos setpoints
        #self.pos_thread = Thread(target=self.send_mission_pos, args=())
        self.pos_thread = Thread(target=self.send_sp_by_state, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
        

    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("STATUS: VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("STATUS: landed state changed from {0} to {1}".format(
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
            rospy.loginfo("STATUS: armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("STATUS: connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("STATUS: mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("STATUS: system_status changed from {0} to {1}".format(
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
        self.suction_pressure = data.data
    
    def winch_state_callback(self, data):
        self.winch_done.value = data.data

    def tfmini_callback(self, data):
        self.tfmini_range.value = data.range
        if self.tfmini_range.value < 0:
            self.tfmini_range.value = 0
    #
    # Helper methods
    #

    # constantly publish waypoint (position / velocity) in a thread
    def send_mission_pos(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() or not self.user_interrupted.value:
            # by default publish zero velocity setpoint as flying is done by manual
            if not self.publish_att_raw.value:
                # publish velocity setpoint for slowly closing / retracting from the wall
                self.pos_target_setpoint_pub.publish(self.make_pos_target()) 
            else:
                # publish attitude setpoint for high attitude movement and takeoff from wall
                self.att_raw_setpoint_pub.publish(self.make_att_target())              
               
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("ERROR: Thread interrupted!")
                pass


    def make_pos_target(self):
        '''
        Create velocity setpoint for approaching the drone slowly to the wall under VICON
        Return: PositionTarget() with vx, vy and vz
        '''
        pos_target = PositionTarget()
        pos_target.header = Header()        
        pos_target.header.frame_id = "sttionary velocity setpoint"
        pos_target.header.stamp = rospy.Time.now()
        pos_target.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                               PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_PX + \
                               PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
        pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pos_target.velocity.x = 0
        pos_target.velocity.y = 0
        pos_target.velocity.z = 0
 
        pos_target.yaw = 0 # don't yaw, always point to the front
        return pos_target
    

    def make_stationary_pos_target(self):
        '''
        Create stationary setpoint_raw for hovering stationary
        Return: PositionTarget() with vx, vy and vz = 0
        '''
        pos_target = PositionTarget()
        pos_target.header = Header()        
        pos_target.header.frame_id = "setpoint_raw/local_stationary"
        pos_target.header.stamp = rospy.Time.now()
        pos_target.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                               PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                               PositionTarget.IGNORE_YAW_RATE
        pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pos_target.velocity.x = 0
        pos_target.velocity.y = 0
        pos_target.velocity.z = 0
        pos_target.yaw = 0 # don't yaw, always point to the front
        return pos_target


    def make_stationary_att_target(self):
        att_target = AttitudeTarget()
        att_target.header = Header()
        att_target.header.stamp = rospy.Time.now()
        att_target.header.frame_id = "stationary_att_target_with_thrust"
                
        att_target.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE
        quaternion = quaternion_from_euler(0, 0, 0)        
        att_target.thrust = 0.73 #self.current_throttle.value
        att_target.orientation = Quaternion(*quaternion)
        return att_target

    def make_pitch_att_target(self):
        att_target = AttitudeTarget()
        att_target.header = Header()
        att_target.header.stamp = rospy.Time.now()
        att_target.header.frame_id = "high_pitch"
        #att_target.header.frame_id = "high_pitch_takeoff"
        # change these values by experiment
        roll_rate = 0.0
        yaw_rate = 0.0
        pitch_rate = self.target_pitch_rate.value      
        att_target.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        att_target.thrust = self.current_throttle.value
        att_target.body_rate = Vector3(x=roll_rate , y=pitch_rate , z=yaw_rate)
        return att_target
        
    def make_att_target(self):
        att_target = AttitudeTarget()
        att_target.header = Header()
        att_target.header.stamp = rospy.Time.now()
        # change these values by experiment
        roll = 0.0
        yaw = 0.0
        pitch = 0.0       
        
        # Throttling up for take off from wall
        # throttle value updated from mission loop
        if self.publish_thr_up.value:
            pitch = 0.0
            att_target.header.frame_id = "throttle_up_from_vertical"
            throttle = self.current_throttle.value
            att_target.type_mask = AttitudeTarget.IGNORE_ATTITUDE
            vector3 = Vector3(x=roll , y=pitch , z=yaw)
            att_target.thrust = throttle
            att_target.body_rate = vector3
            return att_target
        
        # Waiting for suction pressure returning to normal for detach
        # Compensate throttle for reduced lift due to wall friction
        if self.stationary.value:
            return self.make_stationary_att_target()
        
        # 
        if not self.publish_thr_down.value:
            pitch = self.pitch_rate # tested in jmavsim for -ve value = pitch up (flip backward)
            self.current_throttle.value = self.low_throttle_value
            att_target.header.frame_id = "high_pitch_low_throttle"
        else:
            if self.throttle_down_start_time > 0:
                # gradually throttling down
                pitch = -0.2 #0.0
                att_target.header.frame_id = "throttling_to_zero"
                
            else:
                pitch = 0.0  # zero pitch rate 
                att_target.header.frame_id = "zero_throttle"
                #throttle = 0.0
                self.current_throttle.value = 0.0

        throttle = self.current_throttle.value
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

    def run_mission_manual(self):
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        
        # start publishing of velocity setpoint for running of offboard flight mode 
        self.publish_att_raw.value = False
        
        # wait for OFFBOARD mode
        if not self.wait_for_offboard_cmd():
            return False
        
        # check suction pressure and wall perching
        if not self.perch_wall(self.perch_time):
            return False
            
        # begin landing on wall
        if not self.land_on_wall(self.land_on_wall_time, self.throttle_down_time):
            return False
        else:    
            self.vtol.value = True
            self.land()
            rospy.loginfo("STATUS: Wait for 10 sec for landing to complete")
            rospy.sleep(10) # wait for 10 sec here for landing
            rospy.loginfo("STATUS: Checking landed state now....")
            self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
            rospy.loginfo("STATUS: Landed on vertical surface successfully! Disarm now.")
            # disarm the drone
            self.set_arm(False, 5)
            #rospy.loginfo("STATUS: Wait for 3 sec for disarm")
            #rospy.sleep(3)
            while not self.is_mav_state_standby():
                rospy.loginfo("STATUS: Waiting for MAV_STATE_STANDBY...")
                try: 
                    rate.sleep()
                except rospy.ROSInterruptException:
                    rospy.loginfo("ERROR: User interrupt while waiting for MAV_STATE_STANDBY!")
                    break
            rospy.loginfo("STATUS: MAV_STATE_STANDBY now! ")
            rospy.loginfo("STATUS: ============== Landed on wall. Ready to deploy sensor ================= ")
            
        
        done_sensor = False
        rate = rospy.Rate(1)
        while not done_sensor:
            rospy.loginfo("STATUS: You can deploy the sensor now..waiting for finish deployment.")
            if self.winch_done.value:
                done_sensor = True
            try: 
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("ERROR: User interrupt while waiting for sensor deployment!")
                break
            
        if not done_sensor:
            return False    

        rospy.loginfo("="*40)
        rospy.loginfo("STATUS: Finish deployment with sensor. Start to takeoff from the wall.")
        if not self.takeoff_from_wall():
            return False

        rospy.loginfo("STATUS: Re-takeoff is successful. Now detach from wall!")
        
        if not self.detach_from_wall():
            rospy.loginfo("STAUTS: Detach is not successful! Stop here!")
            return False
            
        self.publish_att_raw.value = False
        rospy.loginfo("STATUS: Detach is successful. Fly back by Position mode manually! Program stop!")    



    # sending setpoint command for testing wall take-off in a thread
    def send_sp_by_state(self):
        rate = rospy.Rate(20)   # higher rate may be desired
        while not rospy.is_shutdown() and not self.user_interrupted.value:
            # by default publish zero velocity setpoint as flying is done by manual
            if self.current_state.value == self.STATIONARY_HORIZONTAL:
                self.pos_target_setpoint_pub.publish(self.make_stationary_pos_target())              
            elif self.current_state.value == self.PITCH_TO_HORIZONTAL:
                self.att_raw_setpoint_pub.publish(self.make_pitch_att_target())              
                pass
            else:
                pass

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSException:
                rospy.loginfo("ERROR: Thread exception!")
                pass

    def run_magnet_test(self):
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        
        self.publish_att_raw.value = True
        
        # wait for OFFBOARD mode
        if not self.wait_for_offboard_cmd():
            return False
        
        rospy.loginfo("STATUS: Start to takeoff from the wall.")
        if not self.take_off_test():
            return False
            
        rospy.loginfo("="*40)

        # begin landing on wall
        if not self.land_test(self.land_on_wall_time, self.throttle_down_time):
            return False
        else:    
            self.vtol.value = True
            self.land()
            rospy.loginfo("STATUS: Wait for 5 sec for landing to complete")
            rospy.sleep(5) # wait for 10 sec here for landing
            rospy.loginfo("STATUS: Checking landed state now....")
            self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
            rospy.loginfo("STATUS: Landed on vertical surface successfully! Disarm now.")
            # disarm the drone
            self.set_arm(False, 5)
            #while not self.is_mav_state_standby():
            #    rospy.loginfo("STATUS: Waiting for MAV_STATE_STANDBY...")
            #    try: 
            #        rate.sleep()
            #    except rospy.ROSInterruptException:
            #        rospy.loginfo("ERROR: User interrupt while waiting for MAV_STATE_STANDBY!")
            #        break
            #rospy.loginfo("STATUS: MAV_STATE_STANDBY now! ")
            rospy.loginfo("STATUS: ============== Landed on wall. Ready to deploy sensor ================= ")
            
            
        rospy.loginfo("STATUS: Test is complete! Program stop!")    

    def is_high_attitude(self):
        rospy.loginfo("IMU data.y = {0}".format(self.imu_data.orientation.y))
        return self.imu_data.orientation.y > 0.2 and self.imu_data.orientation.y < 0.7
        
    def is_normal_attitude(self):
        rospy.loginfo("IMU data.y = {0}".format(self.imu_data.orientation.y))
        return self.imu_data.orientation.y < 0.07 #0.15

    def is_vertical_takeoff_attitude(self):
        rospy.loginfo("IMU data.y = {0}".format(self.imu_data.orientation.y))
        return self.imu_data.orientation.y < 0.52
        
    def is_landed_state_on_ground(self):
        # check whether the drone has landed 
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def is_mav_state_standby(self):        
        return mavutil.mavlink.enums['MAV_STATE'][self.state.system_status].name == "MAV_STATE_STANDBY"


    def make_vel_setpoint(self, vx=0.0, vy=0.0, vz=0.0):
        '''
        Create velocity setpoint for approaching the drone slowly to the wall under VICON
        Return: PositionTarget() with vx, vy and vz
        '''
        pos_target = PositionTarget()
        pos_target.header = Header()        
        pos_target.header.frame_id = "zero_vel_sp"
        pos_target.header.stamp = rospy.Time.now()
        pos_target.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                               PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_PX + \
                               PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
        pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pos_target.velocity.x = vx
        pos_target.velocity.y = vy
        pos_target.velocity.z = vz
        pos_target.yaw = 0.0 # don't yaw, always point to the front
        return pos_target

    def pitch_test(self, timeout=60, throttle_timeout=30):
        rospy.loginfo("=================== This is a take-off from wall test ========================")
        rospy.loginfo("STATUS: Set to PITCH_TO_VERTICAL state and OFFBOARD mode.")
        self.current_state.value = self.PITCH_TO_HORIZONTAL
        self.set_mode("OFFBOARD", 5)
        rospy.loginfo("STATUS: Rearm the drone in vertical pose.")
        self.set_arm(True, 5)

        self.target_pitch_rate.value = 0.00

        start_throttle = 0.01
        end_throttle = 0.2 ### 0.2 for empty loading # self.low_throttle_value         
        #rospy.loginfo("STATUS: Throttle set from {0} to {1}".format(start_throttle, end_throttle))
        
        self.current_throttle.value = start_throttle
        self.throttle_up_start_time = rospy.get_time()

        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        period = throttle_timeout * loop_freq 

        throttle_step = (end_throttle - start_throttle) / (period/3.0)


        takeoff_from_vertical = False

        for i in xrange(period):
            rospy.loginfo("STATUS: Auto_throttling up from {0}. current throttle = {1}".format(start_throttle, self.current_throttle.value))
            try:
                # throttling up gradually
                self.current_throttle.value += throttle_step
                # clip max throttle value
                if self.current_throttle.value >= end_throttle:
                    self.current_throttle.value = end_throttle

                # detect SUCTION_IS_LAND param while throttling up
                res = self.get_param_srv('SUCTION_IS_LAND')
                if res.success and res.value.integer <= 0:
                    rospy.loginfo(
                        "VERTICAL_LAND received {0}. drone takes off vertically from the wall! ".format(res.value.integer))
                    takeoff_from_vertical = True
                    break
                    
                rate.sleep()
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                # TODO: handling of throttle value under failure
                rospy.loginfo("STATUS: Auto_throttling is interrupted!")
                self.publish_att_raw.value = True
                self.current_throttle.value = 0.0
                self.user_interrupted.value = True
                break

        if not takeoff_from_vertical:
            self.assertTrue(takeoff_from_vertical, (
                "took too long to take off from wall | timeout(seconds): {0}".format(timeout)))
            self.current_throttle.value = 0.0
            #self.user_interrupted.value = True
            return False
        
        rospy.loginfo("STATUS: Reducing throttle to zero!")
        for i in xrange(period):
            rospy.loginfo("STATUS: Auto_throttling down from {0}. current throttle = {1}".format(end_throttle, self.current_throttle.value))
            try:
                # throttling up gradually
                self.current_throttle.value -= throttle_step
                # clip min throttle value
                if self.current_throttle.value <= 0.0:
                    self.current_throttle.value = 0.0
                    break
                    
                rate.sleep()
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                # TODO: handling of throttle value under failure
                rospy.loginfo("STATUS: Auto_throttling is interrupted!")
                self.current_throttle.value = 0.0
                self.user_interrupted.value = True
                break
        rospy.loginfo("STATUS: Test end!")


    def take_off_test(self, timeout=60, throttle_timeout=60):
        rospy.loginfo("=================== This is a take-off from wall test ========================")
        rospy.loginfo("STATUS: Set to PITCH_TO_VERTICAL state and OFFBOARD mode.")
        self.current_state.value = self.PITCH_TO_VERTICAL
        self.set_mode("OFFBOARD", 5)
        rospy.loginfo("STATUS: Rearm the drone in vertical pose.")
        self.set_arm(True, 5)

        start_throttle = 0.01
        end_throttle = 0.2 ### 0.2 for empty loading # self.low_throttle_value         
        #rospy.loginfo("STATUS: Throttle set from {0} to {1}".format(start_throttle, end_throttle))
        
        self.current_throttle.value = start_throttle
        self.throttle_up_start_time = rospy.get_time()

        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        period = throttle_timeout * loop_freq 
        takeoff_from_vertical = False

        for i in xrange(period):
            rospy.loginfo("STATUS: Auto_throttling up from {0}. current throttle = {1}".format(start_throttle, self.current_throttle.value))
            try:
                # throttling up gradually
                self.current_throttle.value += 0.01 
                if self.current_throttle.value >= end_throttle:
                    self.current_throttle.value = end_throttle

                # detect SUCTION_IS_LAND param while throttling up
                res = self.get_param_srv('VERTICAL_LAND')
                if res.success and res.value.integer <= 0:
                    rospy.loginfo(
                        "VERTICAL_LAND received {0}. drone takes off vertically from the wall! ".format(res.value.integer))
                    takeoff_from_vertical = True
                    break
                    
                rate.sleep()
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                # TODO: handling of throttle value under failure
                rospy.loginfo("STATUS: Auto_throttling is interrupted!")
                self.publish_att_raw.value = True
                self.current_throttle.value = 0.0
                break

        if not takeoff_from_vertical:
            self.assertTrue(takeoff_from_vertical, (
                "took too long to take off from wall | timeout(seconds): {0}".format(timeout)))
            self.publish_att_raw.value = True
            self.current_throttle.value = 0.0
            return False

        start_throttle = self.current_throttle.value
        end_throttle = self.holding_throttle #0.85 for sensor ###0.55 # 0.55 for without loading at the end
        period = timeout * loop_freq // 2
        normal_attitude = False
        rospy.loginfo("Gradually throttling up to normal attitude.")

        while not normal_attitude:
            rospy.loginfo("STATUS: waiting for normal attitude. Current throttle: {0} | IMU-Y = {1}".format(self.current_throttle.value, self.imu_data.orientation.y))
            self.current_throttle.value +=  0.01
            normal_attitude = self.is_normal_attitude()
            rate.sleep()
            
            # handle exception when throttle is too high but normal attitude is not reached, i.e. drone too heavy
            if self.current_throttle.value >= end_throttle:
                rospy.loginfo("STATUS: End throttle value of {0} is reached. Stop throttle there".format(end_throttle))
                self.publish_att_raw.value = True
                self.publish_thr_up.value = False
                self.current_throttle.value = 0               
                break
        
        if normal_attitude:  
            rospy.loginfo("****** STATUS: normal attitude is reached! ready to throttle down again!! ******")
            self.publish_att_raw.value = True
            self.publish_thr_up.value = False
        else:
            rospy.loginfo("STATUS: Drone too heavy / cannot reach the normal attitude!")
            self.publish_att_raw.value = True
            self.publish_thr_up.value = False
            self.current_throttle.value = 0
            return False

        return normal_attitude


    def land_test(self, timeout=60, throttle_timeout=5):
        rospy.loginfo(
             "========= This is a landing test =========")
        rospy.loginfo(
            "Throttle down from is about to start... Throttling down from {0}".format(self.current_throttle.value))
        self.publish_att_raw.value = True
        self.publish_thr_down.value = True
        self.throttle_down_start_time = rospy.get_time()
        ##self.current_throttle.value = self.low_throttle_value
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)

        # gradually reduce throttle to zero during throttle_timeout
        for i in xrange(throttle_timeout * loop_freq):
            try:            
                self.current_throttle.value -= 0.005
                if self.current_throttle.value < 0:
                    self.current_throttle.value = 0
                    rospy.loginfo("STATUS: Throttle already zero!")
                    
                # detect SUCTION_IS_LAND param while throttling down
                res = self.get_param_srv('VERTICAL_LAND')
                if res.success and res.value.integer > 0:
                    rospy.loginfo(
                        "VERTICAL_LAND received {0}. drone's pose is vertical! ".format(res.value.integer))
                    vertical_landing = True
                    # break and set throttle = 0 all the way
                    self.throttle_down_start_time = -1
                    break
                    
                rate.sleep()
                rospy.loginfo("Set Throttle = 0 during vertical landing. Current throttle = {0}".format(self.current_throttle.value))
            except rospy.ROSException as e:
                pass
            except rospy.ROSInterruptException:
                vertical_landing = False
                self.current_throttle.value = 0.0
                rospy.loginfo("ERROR: User interrupt!")
                break

        if not vertical_landing:
            self.assertTrue(vertical_landing, (
                "took too long to reach high attitude | timeout(seconds): {0}".format(timeout)))
            self.publish_thr_down.value = False
            self.publish_att_raw.value = False
        else:
            rospy.loginfo("Near vertical landing to the wall! Start AUTO.LAND now.")

        self.current_throttle.value = 0.0
        return vertical_landing



                
    def takeoff_from_wall(self, timeout=60, throttle_timeout=15):
        rospy.loginfo("================================================================")
        rospy.loginfo("STATUS: Set OFFBOARD mode.")
        self.set_mode("OFFBOARD", 5)
        rospy.loginfo("STATUS: Rearm the drone in vertical pose.")
        self.set_arm(True, 5)
        self.publish_att_raw.value = True
        self.publish_thr_up.value = True

        start_throttle = 0.01
        end_throttle = 0.2 ### 0.2 for empty loading # self.low_throttle_value         
        rospy.loginfo("STATUS: Throttle set from {0} to {1}".format(start_throttle, end_throttle))
        
        self.current_throttle.value = start_throttle
        self.throttle_up_start_time = rospy.get_time()

        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        period = throttle_timeout * loop_freq 
        takeoff_from_vertical = False

        for i in xrange(period):
            rospy.loginfo("STATUS: Auto_throttling up from {0}. current throttle = {1}".format(start_throttle, self.current_throttle.value))
            try:
                # throttling up gradually
                self.current_throttle.value += 0.01 
                if self.current_throttle.value >= end_throttle:
                    self.current_throttle.value = end_throttle

                # detect SUCTION_IS_LAND param while throttling up
                res = self.get_param_srv('VERTICAL_LAND')
                if res.success and res.value.integer <= 0:
                    rospy.loginfo(
                        "VERTICAL_LAND received {0}. drone takes off vertically from the wall! ".format(res.value.integer))
                    takeoff_from_vertical = True
                    break

                # break out if end_throttle value is reached (for autonomous takeoff with distance measurement sensor)
                #if self.current_throttle.value >= end_throttle:
                #    rospy.loginfo("End Throttle 0.3 is reached!")
                #    break
                    
                rate.sleep()
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                # TODO: handling of throttle value under failure
                rospy.loginfo("STATUS: Auto_throttling is interrupted!")
                self.publish_att_raw.value = True
                self.current_throttle.value = 0.0
                break

        if not takeoff_from_vertical:
            self.assertTrue(takeoff_from_vertical, (
                "took too long to take off from wall | timeout(seconds): {0}".format(timeout)))
            self.publish_att_raw.value = True
            self.current_throttle.value = 0.0
            return False

        start_throttle = self.current_throttle.value
        end_throttle = self.holding_throttle #0.85 for sensor ###0.55 # 0.55 for without loading at the end
        period = timeout * loop_freq // 2
        normal_attitude = False
        rospy.loginfo("Gradually throttling up to normal attitude.")

        while not normal_attitude:
            rospy.loginfo("STATUS: waiting for normal attitude. Current throttle: {0} | IMU-Y = {1}".format(self.current_throttle.value, self.imu_data.orientation.y))
            self.current_throttle.value +=  0.01
            normal_attitude = self.is_normal_attitude()
            rate.sleep()
            
            # handle exception when throttle is too high but normal attitude is not reached, i.e. drone too heavy
            if self.current_throttle.value >= end_throttle:
                rospy.loginfo("STATUS: End throttle value of {0} is reached. Stop throttle there".format(end_throttle))
                break
        
        if normal_attitude:  
            rospy.loginfo("STATUS: normal attitude is reached! ready to detach from wall!")
            rospy.loginfo("STATUS: publish stationary_att_throttle for stabilization during detach")
            self.publish_att_raw.value = True
            self.publish_thr_up.value = False
        else:
            rospy.loginfo("STATUS: Drone too heavy / cannot reach the normal attitude!")
            self.publish_att_raw.value = True
            self.publish_thr_up.value = False
            self.current_throttle.value = 0
            return False

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

        for i in xrange(timeout * loop_freq, 0, -1):
            rospy.loginfo(
                        "** High Attitude Movement. Count-down to end: {0}. Current throttle: {1}".format(i, self.current_throttle.value))
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
            "Throttle down from high attitude is about to start... Throttling down from {0}".format(self.low_throttle_value))
        self.publish_thr_down.value = True
        self.throttle_down_start_time = rospy.get_time()
        self.current_throttle.value = self.low_throttle_value
        
        # gradually reduce throttle to zero during throttle_timeout
        for i in xrange(throttle_timeout * loop_freq):
            try:
                # check distance from wall by tfmini range sensor:
                rospy.loginfo("STATUS: Current distance to wall (m): {0}".format(self.tfmini_range.value))
                if self.tfmini_range.value < self.min_distance_from_wall:
                    rospy.loginfo("Drone is close enough to the wall!")
            
                self.current_throttle.value -= 0.005
                if self.current_throttle.value < 0:
                    self.current_throttle.value = 0
                    rospy.loginfo("STATUS: Throttle already zero!")
                    
                # detect SUCTION_IS_LAND param while throttling down
                res = self.get_param_srv('SUCTION_IS_LAND')
                if res.success and res.value.integer > 0:
                    rospy.loginfo(
                        "SUCTION_IS_LAND received {0}. drone's pose is vertical! ".format(res.value.integer))
                    vertical_landing = True
                    # break and set throttle = 0 all the way
                    self.throttle_down_start_time = -1
                    break
                    
                rate.sleep()
                rospy.loginfo("Set Throttle = 0 during vetical landing. Current throttle = {0}".format(self.current_throttle.value))
            except rospy.ROSException as e:
                pass
            except rospy.ROSInterruptException:
                vertical_landing = False
                self.current_throttle.value = 0.0
                rospy.loginfo("ERROR: User interrupt!")
                break

        if not vertical_landing:
            self.assertTrue(vertical_landing, (
                "took too long to reach high attitude | timeout(seconds): {0}".format(timeout)))
            self.publish_thr_down.value = False
            self.publish_att_raw.value = False
        else:
            rospy.loginfo("Near vertical landing to the wall! Start AUTO.LAND now.")

        return vertical_landing
    
    def detach_from_wall(self, timeout=20):
        '''
        Detach the drone from the wall. Precondition: the drone returns to normal attitude after re-takeoff
        return successful detachment or not
        '''
        
        rospy.loginfo("STATUS: Begin detaching from wall")

        # check suction pressure in a loop until suction cup is detached from the wall
        # does it perch to the wall in 'timeout' seconds?
        rospy.loginfo("========= waiting for SUCTION_IS_PERCH for detach =========")
        loop_freq = 5  # Hz
        rate = rospy.Rate(loop_freq)
        detach = False
        
        stationary_period = 6 # sec
        self.publish_att_raw.value = True
        self.stationary.value = True
        
        start_time = rospy.get_time()
        
        while not detach:
            rospy.loginfo(
                        "waiting for SUCTION_IS_PERCH to 0. Current Pressure = {0}".format(self.suction_pressure))            
            try:
                res = self.get_param_srv('SUCTION_IS_PERCH')
                if res.success and res.value.integer <= 0:
                    rospy.loginfo(
                        "SUCTION_IS_PERCH received {0}. drone detaches from the wall! ".format(res.value.integer))
                    detach = True
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)
                break
            try:
                rate.sleep()
            except (rospy.ROSException, rospy.InterruptException) as e:
                self.fail(e)
                rospy.loginfo("ERROR: User Interruption / Exception occurs")
                break
                            
        if detach:
            self.publish_att_raw.value = False
            self.stationary.value = False
            rospy.loginfo("STATUS: Successfully detach from wall manually!")
        else:
            self.publish_att_raw.value = True
            self.stationary.value = True
            self.current_throttle.value = 0.0
            
        return detach

    def perch_wall(self, timeout=60,):
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
            except rospy.ROSInterruptException:
                rospy.loginfo("User interruption")
                pass
        
        self.assertTrue(suction, (
            "took too long to get suction! | timeout(seconds): {0}".format(timeout)))
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

    def wait_for_offboard_cmd(self):
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        offboard_mode = False
        rospy.loginfo("waiting for offboard command. Current State: {0}".format(self.state.mode))
        
        while not offboard_mode:
            if self.state.mode == "OFFBOARD":
                offboard_mode = True

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
            except rospy.ROSInterruptException:
                rospy.loginfo("ERROR: Interruption possibly by user / external. Quit now!")
                offboard_mode = False
                break
                
        return offboard_mode


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
    mode_group.add_argument('-manual', '--manual-test', action='store_true', help="rearm the drone after landing to the wall")
    mode_group.add_argument('-magnet', '--magnet-test', action='store_true', help="rearm the drone after landing to the wall")


    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('suction_mission_node')
    #signal.signal(signal.SIGINT, sigint_handler)
    #signal.signal(signal.SIGTERM, sigint_handler)


    mission_pos_manual = ( (0, 0, 0, 1) )
    
    global suction_mission

    if args.manual_test:
        suction_mission = MavrosOffboardSuctionMission(radius=0.1,
                                                       mission_pos=mission_pos_manual,
                                                       goto_pos_time=60, perch_time=80, land_on_wall_time=60, throttle_down_time=40)
        suction_mission.run_mission_manual()
    elif args.magnet_test:
        suction_mission = MavrosOffboardSuctionMission(radius=0.4,
                                                       mission_pos=mission_pos_manual,
                                                       goto_pos_time=60, perch_time=80, land_on_wall_time=60, throttle_down_time=40)
        suction_mission.pitch_test()


    rospy.spin()


