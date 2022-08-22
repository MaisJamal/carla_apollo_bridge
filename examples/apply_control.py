#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.


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
import math
import time
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')



from cyber_py import cyber, cyber_time
#from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.planning.proto.planning_pb2 import ADCTrajectory

# ==============================================================================
# ------- Cyber Nodes ----------------------------------------------------------
# ==============================================================================





class ApolloFeatures:

    def __init__(self,world):
        cyber.init()
        self.sim_world = world
        self.map = world.get_map()
        self.planned_trajectory = None
        self.player = self.sim_world.get_actors().filter('vehicle.lincoln.mkz*')[0]
        self.node = cyber.Node("carla_control_listner_node")
        self.control_reader = self.node.create_reader('/apollo/control', ControlCommand, self.control_callback)
        #self.planning_reader = self.node.create_reader('/apollo/planning', ADCTrajectory, self.planning_callback)

    def control_callback (self,data):
        vehicle_control = carla.VehicleControl()
        #self.carla_actor.set_simulate_physics(True)
        vehicle_control.hand_brake = data.parking_brake
        vehicle_control.brake = data.brake / 100.0 #/2
        vehicle_control.steer = -1 * data.steering_target / 100.0 
        vehicle_control.throttle = data.throttle / 100.0 #/1.7
        vehicle_control.reverse = data.gear_location == Chassis.GearPosition.GEAR_REVERSE
        self.player.apply_control(vehicle_control)


    def planning_callback(self, msg):
        self.planned_trajectory = ADCTrajectory()
        self.planned_trajectory.CopyFrom(msg)
        #self.move_along_planned_trajectory()


    def move_along_planned_trajectory(self):
        if self.planned_trajectory is None:
            print("empty trajectory")
            return    
        print("planned trajectory is recieved...")
        t = 0
        self.player.set_simulate_physics(False)
        timestamp = cyber_time.Time.now().to_sec()
        transform = self.player.get_transform()
        wp = self.map.get_waypoint(transform.location)
        dt = timestamp - self.planned_trajectory.header.timestamp_sec
        """
        for tp in self.planned_trajectory.trajectory_point:
            x_carla = transform.location.x
            y_carla = transform.location.y
            x_path = tp.path_point.x
            y_path = -tp.path_point.y


            if math.sqrt((x_carla - x_path)**2 + (y_carla - y_path)**2)  <0.01 :
                print("is less")
                t = tp.relative_time
                
        """
        for tp in self.planned_trajectory.trajectory_point:
            
            print("x  " ,tp.path_point.x, "  y  ", tp.path_point.y," t ",tp.relative_time)#," s ", tp.s)
            #if t<= dt < tp.relative_time:
            if 0.005 < tp.relative_time < 0.025 :
                # place car a bit above the ground
                # especially needed in Town02 where the car goes under the map
                height_buffer = 0.1
                #TODO: linear interpolation here
                transform.location.x = tp.path_point.x
                transform.location.y = -tp.path_point.y
                transform.location.z = wp.transform.location.z + height_buffer
                transform.rotation.yaw = -math.degrees(tp.path_point.theta)
                self.player.set_transform(transform)
                return
            

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')

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
    
    args = argparser.parse_args()

    try:

        client = carla.Client(args.host, args.port)
        client.set_timeout(200.0)
        sim_world = client.get_world()
        sim_world.wait_for_tick(5.0)
        vehicle = sim_world.get_actors().filter('vehicle.*')[0]
        print("vehicles are ", vehicle)
        apollo_control_listener = ApolloFeatures(sim_world)
        apollo_control_listener.node.spin()
        #while not cyber.is_shutdown() :
            #apollo_control_listener.move_along_planned_trajectory()
            #time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
