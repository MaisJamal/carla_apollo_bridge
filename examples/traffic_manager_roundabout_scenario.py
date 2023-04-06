#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


import time
from cyber_py import cyber, cyber_time
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.transform.proto.transform_pb2 import TransformStamped, TransformStampeds,Transform
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.drivers.gnss.proto.imu_pb2 import Imu
from modules.localization.proto.gps_pb2 import Gps
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles

#from modules.localization.proto.clock_pb2 import Clock
#from carla_common.clock_pb2 import Clock

#from transforms3d.euler import euler2mat, quat2euler, euler2quat
from carla_common.euler import euler2mat, quat2euler, euler2quat

from scipy.spatial.transform import Rotation as R

global_counter_inner = 0
global_counter_outer = 0#1.57   # pi/2
# ==============================================================================
# ------- Cyber Nodes ----------------------------------------------------------
# ==============================================================================





class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("carla_writer_node")
        self.local_sequence_num = 1
        self.chassis_sequence_num = 1
        self.lidar_sequence_num = 1
        self.imu_sequence_num = 1
        self.gps_sequence_num = 1
        self.obstacles_sequence_num = 1
        self.planned_trajectory = None
        self.location_writer = self.node.create_writer('/apollo/localization/pose', LocalizationEstimate)
        self.chassis_writer = self.node.create_writer('/apollo/canbus/chassis', Chassis)
        self.transform_writer = self.node.create_writer('/tf', TransformStampeds)
        self.lidar_writer = self.node.create_writer('/apollo/sensor/lidar128/compensator/PointCloud2',PointCloud)
        self.imu_writer = self.node.create_writer('/apollo/sensor/gnss/imu',Imu)
        self.gps_writer = self.node.create_writer('/apollo/sensor/gnss/odometry',Gps)
        self.obstacles_writer = self.node.create_writer('/apollo/perception/obstacles',PerceptionObstacles)
        #self.clock_writer = self.node.create_writer('/clock',Clock)
        #self.location_reader = self.reader_node.create_reader('/apollo/control', ControlCommand, control_callback)

    def send_localization_msg(self,carla_actor_player):
        global current_speed
        #clock_msg = Clock()
        #clock_msg.clock = cyber_time.Time.now()
        #self.clock_writer.write(clock_msg)
        transform = carla_actor_player.get_transform()
        linear_vel = carla_actor_player.get_velocity()
        angular_vel = carla_actor_player.get_angular_velocity()
        accel = carla_actor_player.get_acceleration()
        heading = -math.radians(transform.rotation.yaw)

        localization_msg = LocalizationEstimate()
        
        localization_msg.measurement_time = cyber_time.Time.now().to_sec()
        localization_msg.header.frame_id = 'localization'
        localization_msg.header.sequence_num = self.local_sequence_num
        localization_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()

        ####################### Carla transform.location to Apollo Pose ################################
        self.local_sequence_num += 1
        x = transform.location.x
        y = -transform.location.y
        z = transform.location.z
        shift = 1.355
        localization_msg.pose.position.x = x - shift * math.cos(heading)
        localization_msg.pose.position.y = y - shift * math.sin(heading)
        localization_msg.pose.position.z = z

        #######################################################################################
        ####################### Carla transform.rotation to Apollo Quaternion #################

        roll = math.radians(transform.rotation.roll)
        pitch = -math.radians(transform.rotation.pitch)
        yaw = -math.radians(transform.rotation.yaw )
        yaw_for_orientation = -math.radians(transform.rotation.yaw + 90)  ## for testing  ## checked
        #print(yaw_for_orientation)
        quat = euler2quat(roll, pitch, yaw_for_orientation)  # w , x, y, z
        localization_msg.pose.orientation.qw = quat[0]
        localization_msg.pose.orientation.qx = quat[1]
        localization_msg.pose.orientation.qy = quat[2]
        localization_msg.pose.orientation.qz = quat[3]


        #######################################################################################
        ####################### Carla transform.rotation to Euler angles #################

        #roll_apollo = transform.rotation.roll
        #pitch_apollo = -transform.rotation.pitch
        #yaw_apollo = -(transform.rotation.yaw +90 ) 
        #if roll_apollo < 0 : 
        #    roll_apollo = 360 + roll_apollo     
        #if pitch_apollo < 0 : 
         #   pitch_apollo = 360 + pitch_apollo     
        #if yaw_apollo < 0 : 
        #    yaw_apollo = 360 + yaw_apollo     
        if yaw_for_orientation < 0 :
           yaw_for_euler = 6.28318530718 + yaw_for_orientation
        else:
            yaw_for_euler = yaw_for_orientation

        localization_msg.pose.euler_angles.x = 0 #roll_apollo           ## for testing
        localization_msg.pose.euler_angles.y = 0 #pitch_apollo         ## for testing
        

        localization_msg.pose.euler_angles.z =  yaw_for_euler    ## for testing




        #######################################################################################
        ####################### Carla transform to Apollo transform ###########################
        
        transform_msg = TransformStampeds()
        transform_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        transform_msg.header.frame_id = "world"
        #print("heading: ", heading , " shift on x : ",- shift * math.cos(heading)," shift on y : ",- shift * math.sin(heading))
        child = transform_msg.transforms.add()
        child.child_frame_id = "localization"
        child.transform.translation.x = x - shift * math.cos(heading)
        child.transform.translation.y = y - shift * math.sin(heading)
        child.transform.translation.z = z
        child.transform.rotation.qw = quat[0]
        child.transform.rotation.qx = quat[1]
        child.transform.rotation.qy = quat[2]
        child.transform.rotation.qz = quat[3]
        self.transform_writer.write(transform_msg)

        #######################################################################################
        ####################### Carla car center transform to Apollo car center ###############
        
        """
        transform_msg = TransformStampeds()
        transform_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        transform_msg.header.frame_id = "temp"

        child = transform_msg.transforms.add()
        child.child_frame_id = "localization"
        child.transform.translation.x = - shift * math.cos(heading)
        child.transform.translation.y = - shift * math.sin(heading)
        child.transform.translation.z = 0
        child.transform.rotation.qw = 1
        child.transform.rotation.qx = 0
        child.transform.rotation.qy = 0
        child.transform.rotation.qz = 0
        self.transform_writer.write(transform_msg)
        """

        #######################################################################################
        ####################### Carla velocity to Apollo linear and angular velocity ##########
        ## linear velocity and rotation are needed
        
        numpy_array = euler2mat(roll, pitch, yaw) # roll, pitch, yaw should be in radian
        #numpy_array = euler2mat(roll_apollo, pitch_apollo, yaw_apollo)
        #print(numpy_array)

        rotation_matrix = numpy_array[:3, :3]
        tmp_array = rotation_matrix.dot(np.array([linear_vel.x, linear_vel.y, linear_vel.z]))
        #print(rotation_matrix)
        #minus_90_rotation_matrix = np.array([  [0.0000000, 1.0000000,  0.0000000], [-1.0000000 , 0.0000000,  0.0000000]  ,[  0.0000000,  0.0000000 , 1.0000000 ]])### for testing
        #tmp_array = minus_90_rotation_matrix.dot(tmp_array)  ############# for testing 

        localization_msg.pose.linear_velocity.x = linear_vel.y #tmp_array[0] 
        localization_msg.pose.linear_velocity.y = linear_vel.x #-tmp_array[1]
        localization_msg.pose.linear_velocity.z = linear_vel.z #tmp_array[2]
   

        localization_msg.pose.angular_velocity_vrf.x = 0 #  math.radians(angular_vel.x)
        localization_msg.pose.angular_velocity_vrf.y = 0 # -math.radians(angular_vel.y)
        localization_msg.pose.angular_velocity_vrf.z = 0 # -math.radians(angular_vel.z)
        localization_msg.pose.angular_velocity.x = 0 #  math.radians(angular_vel.x)
        localization_msg.pose.angular_velocity.y = 0 # -math.radians(angular_vel.y)
        localization_msg.pose.angular_velocity.z = 0 # -math.radians( angular_vel.z)
        #######################################################################################


        #print("heading: ", heading, " transform.rotation.yaw: ", transform.rotation.yaw," rad_angular_vel.z: ", math.radians(- angular_vel.z) )
        #localization_msg.pose.position.x = x
        #localization_msg.pose.position.y = y
        #localization_msg.pose.position.z = z
        #localization_msg.pose.linear_velocity.x = linear_vel.x
        #localization_msg.pose.linear_velocity.y = linear_vel.y
        #localization_msg.pose.linear_velocity.z = linear_vel.z
        if accel.y > 2 : 
            accel.y = 2
        elif accel.y < -2:
            accel.y = -2
        if accel.x > 2 : 
            accel.x = 2
        elif accel.x < -2:
            accel.x = -2
        linear_a_x =  accel.y   ## for testing- accel.y   ## for testing
        linear_a_y = accel.x   ## for testing- accel.x   ## for testing
        linear_a_z = accel.z  + 9.8   ## for testing

        localization_msg.pose.linear_acceleration.x = 0 # linear_a_x        ## for testing  / it was accel.x
        localization_msg.pose.linear_acceleration.y = 0 # linear_a_y       ## for testing  / it was -accel.y
        localization_msg.pose.linear_acceleration.z = 0 # linear_a_z  ## for testing

        ################ linear acceleration in vehicle coordination system in Apollo #################

        ### calculate rotation matrix ####
        heading_in_degrees = -1* transform.rotation.yaw
        if heading_in_degrees < 0 :
            heading_in_degrees += 360
        rotation_angle = heading_in_degrees - 90   # rotation angle around z from velocity/acceleration frame to vehicle frame
        ##                         ^ Y'                                 
        ##                         |
        ##                         |         (vehicle frame)            
        ##                         |
        ##         heading <-------o------->X'                        o------->Y                                 
        ##                                                            |
        ##                                                            |               (external frame in which velocity and acceleration are given)            
        ##                                                            |
        ##                                                            v X

        r = R.from_euler('z', rotation_angle, degrees=True)
        #print("rotation angle: ",rotation_angle)
        rot_mat = r.as_dcm()
        #print(rot_mat) # print rotation matrix
        rot_mat = r.inv().as_dcm()
        rotated_a = rot_mat.dot(np.array([linear_a_x, linear_a_y, linear_a_z]))
        localization_msg.pose.linear_acceleration_vrf.x = 0 # rotated_a[0]
        localization_msg.pose.linear_acceleration_vrf.y = 0 # rotated_a[1]
        localization_msg.pose.linear_acceleration_vrf.z = 0 # rotated_a[2]       ## for testing
        localization_msg.pose.heading = heading
        #print("old x : ", x)
        #print("old y : ", y)
        
        #print("new x : ", localization_msg.pose.position.x)
        #print("new y : ", localization_msg.pose.position.y)
        self.location_writer.write(localization_msg)

    
    def send_chassis_msg(self,carla_actor_player):
        v = carla_actor_player.get_velocity()
        c = carla_actor_player.get_control()
        speed_abs = math.sqrt(v.x**2 + v.y**2 + v.z**2)   # in m/s
        #speed_abs = 10
        #print("velocity of vehicle:" , speed_abs)
        chassis_msg = Chassis()
        chassis_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        chassis_msg.header.sequence_num = self.chassis_sequence_num
        self.chassis_sequence_num += 1
        chassis_msg.engine_started = True
        chassis_msg.speed_mps = speed_abs
        chassis_msg.throttle_percentage = c.throttle * 100.0 #*1.5#*2
        chassis_msg.brake_percentage = c.brake * 100.0 
        chassis_msg.steering_percentage = -1* c.steer * 100.0#*1.5#*2
        chassis_msg.parking_brake = c.hand_brake
        chassis_msg.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        if c.reverse :
            chassis_msg.gear_location = Chassis.GearPosition.GEAR_REVERSE
            #print("gear reverse")
        else :
            chassis_msg.gear_location = Chassis.GearPosition.GEAR_DRIVE
            #print("gear drive")


        self.chassis_writer.write(chassis_msg)

    def send_lidar_msg(self,lidar_msg):
        lidar_msg.header.sequence_num = self.lidar_sequence_num
        self.lidar_sequence_num += 1
        self.lidar_writer.write(lidar_msg)
        #print("lidar sent")
    def send_imu_msg(self,imu_msg):
        imu_msg.header.sequence_num = self.imu_sequence_num
        self.imu_sequence_num += 1
        self.imu_writer.write(imu_msg)
    def send_gps_msg(self,gps_msg):
        gps_msg.header.sequence_num = self.gps_sequence_num
        self.gps_sequence_num += 1
        self.gps_writer.write(gps_msg)
    
    def send_obstacles_msg(self,carla_actor_player,world) : 
        obstacles = PerceptionObstacles()
        obstacles.header.timestamp_sec = cyber_time.Time.now().to_sec()
        obstacles.header.sequence_num = self.obstacles_sequence_num
        self.obstacles_sequence_num += 1
        vehicles_num = 0
        walkers_num = 0
        for actor in world.get_actors():
            #print("actor: ",actor[0])
            
            #if actor.carla_actor.get_location().distance(self.parent_actor.get_location()) <= self.range:
            if isinstance(actor, carla.Vehicle) and actor != carla_actor_player:
                vehicles_num +=1
            elif isinstance(actor, carla.Walker):
                walkers_num +=1
                
        #print("walkers: ", walkers_num)
        #print("vehicles: ", vehicles_num)        
        #for actor in self.bridge.child_actors.values():
        """
        
        obstacles.header.CopyFrom(self.bridge.get_cyber_header())
        for actor in self.bridge.child_actors.values():
            if actor.carla_actor is not self.parent_actor:
                print ("actor in object sensore: ", actor.carla_actor)
                print("actor: ", self.get_actor_display_name(actor.carla_actor))
                if actor.carla_actor.get_location().distance(self.parent_actor.get_location()) <= self.range:
                    if isinstance(actor.carla_actor, carla.Vehicle):
                        obstacles.perception_obstacle.append(actor.get_cyber_obstacle_msg())
                    elif isinstance(actor.carla_actor, carla.Walker):
                        msg = actor.get_cyber_obstacle_msg()
                        msg.type = PerceptionObstacle.Type.PEDESTRIAN
                        obstacles.perception_obstacle.append(msg)
        self.bridge.write_cyber_message(self.channel_name, obstacles)
        """  

# ==============================================================================
# ---- Spawn obstacles ---------------------------------------------------------
# ==============================================================================


def add_obstacle (player,world):
    transform = player.get_transform()
    x_player = transform.location.x
    y_player = transform.location.y
    yaw_player = transform.rotation.yaw
    heading = math.radians(yaw_player)
    print("yaw:",yaw_player)
    x_obstacle = x_player + 10 * math.cos(heading)
    y_obstacle = y_player + 10 * math.sin(heading)
    obs_blueprint = world.get_blueprint_library().find('vehicle.citroen.c3')
    spawn_point = carla.Transform(carla.Location(x=x_obstacle,y=y_obstacle, z=0.3), carla.Rotation(yaw= yaw_player))
    obstacle = world.try_spawn_actor(obs_blueprint, spawn_point)

    return obstacle


def add_obstacle_on_roundabout (world,inner):
    # add obstacle on the inner circle
    if inner:
        x_obstacle = -20.0
        y_obstacle = -1.20
        yaw_obstacle = 90
    else:
        x_obstacle = -23.5
        y_obstacle = -1.20
        yaw_obstacle = 90
    obs_blueprint = world.get_blueprint_library().find('vehicle.citroen.c3')
    spawn_point = carla.Transform(carla.Location(x=x_obstacle,y=y_obstacle, z=0.3), carla.Rotation(yaw= yaw_obstacle))
    obstacle = world.try_spawn_actor(obs_blueprint, spawn_point)
    return obstacle

def update_obstacle (obstacle):
    transform = obstacle.get_transform()
    x_obs = transform.location.x
    y_obs = transform.location.y
    z_obs = transform.location.z
    yaw_obs = transform.rotation.yaw
    heading = math.radians(yaw_obs)
    x_obstacle = x_obs + 0.06 * math.cos(heading)
    y_obstacle = y_obs + 0.06 * math.sin(heading)
    new_trans = carla.Transform(carla.Location(x=x_obstacle,y=y_obstacle, z=z_obs), carla.Rotation(yaw= yaw_obs))
    obstacle.set_transform(new_trans)

    return obstacle

def update_obstacle_rotating (obstacle,inner):
    global global_counter_inner
    global global_counter_outer
    transform = obstacle.get_transform()
    x_obs = transform.location.x
    y_obs = transform.location.y
    z_obs = transform.location.z
    yaw_obs = transform.rotation.yaw
    #yaw_obs_rad = math.radians(yaw_obs)
    if inner:
        radius = 19.8
        global_counter = global_counter_inner
    else:
        radius = 23
        global_counter = global_counter_outer
    
    #x_obstacle = x_obs + 0.01 * math.cos(yaw_obs_rad - global_counter)   #rad  // x = x0 + r cost     // r = 20 m //x = r * cos(t) + cx
    x_obstacle = radius * math.cos(global_counter) - 0.1
    #y_obstacle = y_obs + 0.01 * math.sin(yaw_obs_rad - global_counter)   #rad  // y = y0 + r sint  ///y = r * sin(t) + cy
    y_obstacle = radius * math.sin(global_counter) + 0.1
    yaw_obs = math.degrees(- 1.57 + global_counter)
    new_trans = carla.Transform(carla.Location(x=x_obstacle,y=y_obstacle, z=z_obs), carla.Rotation(yaw= yaw_obs ))#- math.degrees(global_counter)
    obstacle.set_transform(new_trans)
    
    if inner:
        global_counter_inner -= 0.001 #counterclockwise
    else:
        global_counter_outer -= 0.003
    return obstacle

# ==============================================================================
# ---- Display Manager ----------------------------------------------------------
# ==============================================================================

class DisplayManager:
    def __init__(self, grid_size, window_size, show_window=False):
        if show_window:
            pygame.init()
            pygame.font.init()
            self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        else:
            self.display = None

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[0]), int(self.window_size[1]/self.grid_size[1])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[0] * dis_size[0]), int(gridPos[1] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

# ==============================================================================
# ---- Sensor Manager ----------------------------------------------------------
# ==============================================================================

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, node):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.cyber_node = node
        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            print("ok")
            lidar_bp.set_attribute('rotation_frequency', '20')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('range', '10000')
            lidar_bp.set_attribute('upper_fov', '1.0')
            lidar_bp.set_attribute('lower_fov', '-20.0')
            lidar_bp.set_attribute('role_name', 'lidar128')
            lidar_bp.set_attribute('points_per_second', '100000')
           
            #lidar_bp.set_attribute('range', '100')
            #lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            #lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            #lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])


            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.send_lidar_msg)

            return lidar
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def send_lidar_msg(self, image):
        #t_start = self.timer.time()
        #print("listening...")
        #disp_size = self.display_man.get_display_size()
        #lidar_range = 2.0*float(self.sensor_options['range'])
        ################
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        #print(points.size)
        msg_data = np.copy(points)
        #msg_data[:, 1] *= -1

        msg_data = -msg_data
        
        msg_data[:,0] *= -1

        lidar_msg = PointCloud()
        lidar_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        lidar_msg.header.frame_id = 'lidar'
        #lidar_msg.frame_id = 'lidar'

        lidar_msg.measurement_time = cyber_time.Time.now().to_sec()
        for lidar_point in msg_data:
            cyber_point = PointXYZIT()
            cyber_point.x = lidar_point[0]
            cyber_point.y = lidar_point[1]
            cyber_point.z = -lidar_point[2]
            cyber_point.intensity = int(-lidar_point[3] * 255)
            #if lidar_point[2] >=0 :
            #    print("positive z.")
            lidar_msg.point.append(cyber_point)

        self.cyber_node.send_lidar_msg(lidar_msg)



    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        #print("Hola, saving Radar data!!")
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud,cyber_node ,args):
        self.cyber_node = cyber_node
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = 'vehicle.lincoln.mkz_2017' #'vehicle.lincoln.mkz_2020'#edited by Mais #args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        ########### added by Mais to check all the available vehicles #############
        #blueprints = [bp for bp in self.world.get_blueprint_library().filter('*')]
        #for blueprint in blueprints:
        #    print(blueprint.id)
        #for attr in blueprint:
        #    print('  - {}'.format(attr))
        ###########################################################################
        blueprint = random.choice(get_actor_blueprints(self.world, self._actor_filter, self._actor_generation))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_point = carla.Transform(carla.Location(x=-6.2,y=-48.9, z=0.3), carla.Rotation(yaw= 90))
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player,self.cyber_node)
        self.imu_sensor = IMUSensor(self.player,self.cyber_node)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma,self.cyber_node)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()
        for actor in self.world.get_actors():
            if isinstance(actor, carla.Vehicle) and actor.attributes.get('role_name') != 'hero':
                actor.destroy() 
            elif isinstance(actor, carla.Walker):
                actor.destroy() 


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = 1.0 # 0.55 #min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = max(self._control.throttle - 0.04, 0)#0 # max(self._control.throttle - 0.01, 0.30)
            
        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = 0.9 #1.0 #0.1 # min(self._control.brake + 0.2, 1)
            if self._control.throttle == 0 :
                self._control.throttle = 0.1
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        #print(world.world.get_actors())

        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor,node):
        self.sensor = None
        self.cyber_node = node
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        x,y = self.from_gps(self.lat, self.lon)

        gps_msg = Gps()

        gps_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        gps_msg.header.frame_id = 'gps'

        gps_msg.localization.position.x = x
        gps_msg.localization.position.y = y
        gps_msg.localization.position.z = -0.7

        self.cyber_node.send_gps_msg(gps_msg)
        #print("GPS  x : ", x , " y : ", y )

    def from_gps(self, latitude, longitude):
        """Creates Location from GPS (latitude, longitude, altitude).

        This is the inverse of the _location_to_gps method found in
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
        """
        EARTH_RADIUS_EQUA = 6378137.0
        # The following reference values are applicable for towns 1 through 7,
        # and are taken from the corresponding CARLA OpenDrive map files.
        LAT_REF = 49.0
        LON_REF = 8.0

        scale = math.cos(LAT_REF * math.pi / 180.0)
        basex = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * LON_REF
        basey = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + LAT_REF) * math.pi / 360.0))

        x = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * longitude - basex
        y = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + latitude) * math.pi / 360.0)) - basey

        # This wasn't in the original carla method, but seems to be necessary.
        y *= -1

        return x,y

# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor,node):
        self.sensor = None
        self.cyber_node = node
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()

        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
        
        imu_msg = Imu()

        imu_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        #imu_msg.header.frame_id = 'imu'
        
        imu_msg.measurement_time = cyber_time.Time.now().to_sec()

        """
        // When measurement_span is non-zero, the gyroscope and accelerometer
        // measurements are averaged for the period from
        // (measurement_time - measurement_span) to measurement_time. Usually,
        //      measurement_span = 1 / sampling_frequency.
        //
        // When measurement_span is 0, angular_velocity and linear_acceleration are
        // instantaneous at measurement_time.
        optional float measurement_span = 3 [default = 0.0];  // In seconds.
        """ 

        imu_msg.measurement_span = 1 / 20.0
          

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        imu_msg.angular_velocity.x = -sensor_data.gyroscope.x
        imu_msg.angular_velocity.y = sensor_data.gyroscope.y
        imu_msg.angular_velocity.z = -sensor_data.gyroscope.z

        imu_msg.linear_acceleration.x = sensor_data.accelerometer.x
        imu_msg.linear_acceleration.y = -sensor_data.accelerometer.y
        imu_msg.linear_acceleration.z = sensor_data.accelerometer.z
        """ can be needed for corrected imu
        roll, pitch, yaw = trans.carla_rotation_to_RPY(sensor_data.transform.rotation)
        [quat_0,quat_1,quat_2,quat_3] = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat_0
        imu_msg.orientation.x = quat_1
        imu_msg.orientation.y = quat_2
        imu_msg.orientation.z = quat_3
        """
        self.cyber_node.send_imu_msg(imu_msg)
           



# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction,cyber_node):
        self.cyber_node = cyber_node
        self.sensor = None
        self.lidar = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '10000'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                #print("lidar:")
                self.lidar_range = 50

                #for attr_name, attr_value in item[3].items():
                 #   bp.set_attribute(attr_name, attr_value)
                 #   if attr_name == 'range':
                 #       self.lidar_range = float(attr_value)
                #        print(attr_value)
               # bp.set_attribute('rotation_frequency', '20')
               # bp.set_attribute('channels', '32')
              #  bp.set_attribute('range', '10000')
              #  bp.set_attribute('upper_fov', '1.0')
             #   bp.set_attribute('lower_fov', '-20.0')
             #   bp.set_attribute('role_name', 'lidar128')
            #    bp.set_attribute('points_per_second', '100000')

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index


    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        #print("parse image is called")
        self = weak_self()
        if not self:
            print("not self")
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))

            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            print("sensor.camera.dvs")

            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            print("sensor.camera.optical_flow")
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()

    #display_manager = None
    #timer = CustomTimer()

    world = None
    original_settings = None
    apollo_test = ApolloFeatures()
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(200.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud,apollo_test, args)
        #world.camera_manager.toggle_camera()
        controller = KeyboardControl(world, args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()

        
        vehicles = sim_world.get_actors().filter('vehicle.lincoln.mkz*')
        ego = vehicles[0]
        #ego.set_enable_gravity(True)
        #ego.set_simulate_physics(True)
        if len(vehicles) > 1 :
            print("Warning: more than one vehicle in the scene.") 

        #'points_per_second': '100000'
        display_manager = DisplayManager(grid_size=[2, 2], window_size=[100, 100])

        sens_manager = SensorManager(sim_world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)), ego, {'range': '10000'} ,[1, 1],apollo_test)
        
        for ac in sim_world.get_actors():
            print("actor: ", ac)



        # Set up the TM in synchronous mode
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # Set a seed so behaviour can be repeated if necessary
        traffic_manager.set_random_device_seed(0)
        random.seed(0)

        waypoints = client.get_world().get_map().generate_waypoints(distance=2.0)
        for i, waypoint in enumerate(waypoints):
            sim_world.debug.draw_string(waypoint.transform.location, str(i), life_time=340)

        spawn_points = sim_world.get_map().get_spawn_points()
        spawn_point_1 =  spawn_points[233]

        route_1_indices = [6931,5044,5054,689,695,2823,2599,2825,6605,6267,6263,1027,2897,6977,6989,3111]
        route_1 = []
        for ind in route_1_indices:
            route_1.append(waypoints[ind].transform.location)
            sim_world.debug.draw_string(waypoints[ind].transform.location, str(ind), life_time=20, color=carla.Color(120,255,0))        

        vehicle_bp = client.get_world().get_blueprint_library().filter('model3')[0]
        vehicle_bp.set_attribute('role_name', 'obstacle')
        # Spawn vehicle 
        vehicle = sim_world.try_spawn_actor(vehicle_bp, spawn_point_1)

        vehicle.set_autopilot(True) # Give TM control over vehicle

        # Set parameters of TM vehicle control, we don't want lane changes
        traffic_manager.update_vehicle_lights(vehicle, True)
        traffic_manager.random_left_lanechange_percentage(vehicle, 0)
        traffic_manager.random_right_lanechange_percentage(vehicle, 0)
        traffic_manager.auto_lane_change(vehicle, True)
        
        # Alternate between routes
        traffic_manager.set_path(vehicle, route_1)
        traffic_manager.vehicle_percentage_speed_difference(vehicle, 60)


        ############# second obstacle ###############        
        route_2_indices = np.linspace(6980,7004,13).astype(int).tolist() + [3110] + np.linspace(6920,6932,7).astype(int).tolist() + np.linspace(5041,5053,7).astype(int).tolist()
        route_2 = []
        first_waypoint = waypoints[route_2_indices[0]].transform
        spawn_point_2 = carla.Transform(carla.Location(x=first_waypoint.location.x,y=first_waypoint.location.y, z=first_waypoint.location.z+2), first_waypoint.rotation)
        # Spawn vehicle 
        vehicle_2 = sim_world.try_spawn_actor(vehicle_bp, spawn_point_2)
        vehicle_2.set_autopilot(True) # Give TM control over vehicle

        traffic_manager.update_vehicle_lights(vehicle_2, True)
        traffic_manager.random_left_lanechange_percentage(vehicle_2, 0)
        traffic_manager.random_right_lanechange_percentage(vehicle_2, 0)
        traffic_manager.auto_lane_change(vehicle_2, True)
        
        # Alternate between routes
        traffic_manager.set_path(vehicle_2, route_2)
        traffic_manager.vehicle_percentage_speed_difference(vehicle_2, 80)### obstacle speed in relation to the lane upper speed limit

        #############################################
        while True:

            #carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0))
            old_loc = ego.get_location()
            loc = carla.Location(x=old_loc.x+0.01,y=old_loc.y,z=old_loc.z)

            old_trans = ego.get_transform()
            old_rot = old_trans.rotation
            new_rot = old_rot
            new_rot.yaw += 0.6
            transf = carla.Transform(old_loc,new_rot)
            #display_manager.render()
            ######ego.set_location(loc)
            #ego.set_transform(transf)
            #update_obstacle(obs1)
            #update_obstacle_rotating(obstacle_2,inner_2)
            #update_obstacle_rotating(obstacle_3,inner_3)
            """
            velocity_in_car_coord = np.array([1,0,0,1])
            Inverse_transform =  np.array(old_trans.get_inverse_matrix())
            tran =  np.array(old_trans.get_matrix())
            velocity_in_world = np.dot(tran, velocity_in_car_coord)
            """

            heading_in_degrees = -1* old_rot.yaw
            if heading_in_degrees < 0 :
                heading_in_degrees += 360
            rotation_angle = heading_in_degrees - 90   

            r = R.from_euler('z', rotation_angle, degrees=True)
            rot_mat = r.as_dcm()
            #print(rot_mat) # print rotation matrix
            rot_mat = r.inv().as_dcm()
            velocity_in_world = rot_mat.dot(np.array([0, -1, 0]))
            #print("velocity: ", velocity_in_world[0]," ", velocity_in_world[1]," ",velocity_in_world[2])
            #ego.set_target_velocity(carla.Vector3D(x=velocity_in_world[0],y=velocity_in_world[1],z=velocity_in_world[2]))
            if args.sync:
                sim_world.tick()
            #if not cyber.is_shutdown():
                #apollo_test.send_localization_msg(ego)
                #apollo_test.send_chassis_msg(ego)
             #   apollo_test.send_obstacles_msg(ego,sim_world)
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock, args.sync):
                return
            #print(.filter('vehicle.*').id())
            
            
            
            ##all_actors = sim_world.get_actors()
            #all_actors = vehicles[0].get_actors()
           ## for act in all_actors:
                #print("actor: ", get_actor_display_name(act))
            ##    print("actor: ", act)
                #children = act.get_actors()
                #for chil in children:
                #    print("his children: ", chil)
            
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            sens_manager.destroy()
            world.destroy()
            

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='172.17.0.1',
        help='IP of the host server (default: 172.17.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    #argparser.add_argument(
    #    '--filter',
     #   metavar='PATTERN',
     #   default='vehicle.*',
     #   help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
