#!/usr/bin/env python

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import yaml
import carla
import math
import time

from cyber_py import cyber, cyber_time , cyber_timer

from msg_getters import get_chassis_msg ,get_localization_msg

from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.transform.proto.transform_pb2 import TransformStamped, TransformStampeds,Transform
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.drivers.gnss.proto.imu_pb2 import Imu
from modules.localization.proto.gps_pb2 import Gps
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.planning.proto.planning_pb2 import ADCTrajectory
# ==============================================================================
# -- global variables ----------------------------------------------------------
# ==============================================================================





# ==============================================================================
# -- Apollo Node ---------------------------------------------------------------
# ==============================================================================


class ApolloNode:

    def __init__(self,world,player,params):
        cyber.init()
        self.sim_world = world
        self.player = player
        self.node = cyber.Node("bridge_node")
        self.params = params
        self.msg_seq_counter = 0
        self.ego_speed = 0
        self.planning_flag = 0

        self.planned_trajectory = None
        self.last_trajectory = None
        self.flag_planning_failed = False

        if self.params['publish_localization_chassis_msgs']:
            self.location_writer = self.node.create_writer(params['localization_channel'], LocalizationEstimate)
            self.chassis_writer = self.node.create_writer(params['chassis_channel'], Chassis)
        if params['apply_control']:
            self.reader = self.node.create_reader(params['control_channel'], ControlCommand, self.control_callback)
        else:
            self.planning_flag = True
            self.reader = self.node.create_reader(params['planning_channel'], ADCTrajectory, self.planning_callback)
    
    def publish_data(self):
        #print("publish is called")
        if self.params['publish_localization_chassis_msgs']:
            chassis_msg = get_chassis_msg(self.player,self.planning_flag,self.ego_speed)
            chassis_msg.header.sequence_num = self.msg_seq_counter
            self.chassis_writer.write(chassis_msg)

            localization_msg = get_localization_msg(self.player)
            localization_msg.header.sequence_num = self.msg_seq_counter
            self.location_writer.write(localization_msg)
        
        self.msg_seq_counter += 1
    
    def planning_callback(self,msg):
        t1 = time.time()
        self.planned_trajectory = ADCTrajectory()
        self.planned_trajectory.CopyFrom(msg)
        ff = 5
        self.player.set_simulate_physics(True)
        temp_empty = False
        temp_stop = False
        temp_plan = False
        if len(self.planned_trajectory.trajectory_point)<1:
            if not self.last_trajectory :
                print("Received trajectory is empty.")
            else:
                #print("Suddenly received trajectory is empty.")
                self.flag_planning_failed = True
                planning_needed = True
                old_trans = self.player.get_transform()
                old_loc = old_trans.location
                old_rot = old_trans.rotation

                dx_nearest_to_vehicle = self.last_trajectory.trajectory_point[0].path_point.x - old_loc.x 
                dy_nearest_to_vehicle = self.last_trajectory.trajectory_point[0].path_point.y - (-old_loc.y)
                nearest_dist = math.sqrt(dx_nearest_to_vehicle * dx_nearest_to_vehicle + dy_nearest_to_vehicle * dy_nearest_to_vehicle)
                nearest_idx = 0

                i=0
                for tp in self.last_trajectory.trajectory_point:
                    dx_current_to_vehicle = self.last_trajectory.trajectory_point[i].path_point.x - old_loc.x 
                    dy_current_to_vehicle = self.last_trajectory.trajectory_point[i].path_point.y - (-old_loc.y)
                    current_dist_to_vehicle = math.sqrt(dx_current_to_vehicle * dx_current_to_vehicle 
                                        + dy_current_to_vehicle * dy_current_to_vehicle)
                    if current_dist_to_vehicle < nearest_dist :
                        nearest_dist = current_dist_to_vehicle
                        nearest_idx = i
                
                    i+=1
                if nearest_idx +ff == i: #last point 
                    nearest_point = self.last_trajectory.trajectory_point[nearest_idx].path_point
                    self.ego_speed = self.last_trajectory.trajectory_point[nearest_idx].v
                else:
                    nearest_point = self.last_trajectory.trajectory_point[nearest_idx+ff].path_point
                    self.ego_speed = self.last_trajectory.trajectory_point[nearest_idx+ff].v
                self.ego_speed = self.last_trajectory.trajectory_point[nearest_idx].v
                #print("v ",self.ego_speed) 
                new_loc = carla.Location(x=nearest_point.x,y=-nearest_point.y,z=old_loc.z)
                new_rot = old_rot
                new_rot.yaw = - math.degrees(nearest_point.theta)
                transf_end = carla.Transform(new_loc,new_rot)
                #self.player.set_target_velocity(carla.Vector3D(x=8,y=0,z=0))
                self.player.set_transform(transf_end)
                #self.send_chassis_msg(speed)
                

        else:
            old_trans = self.player.get_transform()
            old_loc = old_trans.location
            old_rot = old_trans.rotation
            
            #print("current point x : ",old_loc.x , " , y :" , old_loc.y , " , yaw: ", old_rot.yaw)
            
            dx_nearest_to_vehicle = self.planned_trajectory.trajectory_point[0].path_point.x - old_loc.x 
            dy_nearest_to_vehicle = self.planned_trajectory.trajectory_point[0].path_point.y - (-old_loc.y)
            nearest_dist = math.sqrt(dx_nearest_to_vehicle * dx_nearest_to_vehicle + dy_nearest_to_vehicle * dy_nearest_to_vehicle)
            nearest_idx = 0

            i=0
            tt1 = time.time()
            for tp in self.planned_trajectory.trajectory_point:
                dx_current_to_vehicle = self.planned_trajectory.trajectory_point[i].path_point.x - old_loc.x 
                dy_current_to_vehicle = self.planned_trajectory.trajectory_point[i].path_point.y - (-old_loc.y)
                current_dist_to_vehicle = math.sqrt(dx_current_to_vehicle * dx_current_to_vehicle 
                                    + dy_current_to_vehicle * dy_current_to_vehicle)
                if current_dist_to_vehicle < nearest_dist :
                    nearest_dist = current_dist_to_vehicle
                    nearest_idx = i
                #print("i: ", i , " v: ",self.planned_trajectory.trajectory_point[i].v," t: ", self.planned_trajectory.trajectory_point[i].relative_time)
                i+=1
            #print(time.time()-tt1)
            #print("nearest index: ",nearest_idx," of total : ", i)
            curr_nearest_point = self.planned_trajectory.trajectory_point[nearest_idx].path_point
            if nearest_idx +ff == i: #last point 
                nearest_point = curr_nearest_point
                self.ego_speed = self.planned_trajectory.trajectory_point[nearest_idx].v 
            else:
                nearest_point = self.planned_trajectory.trajectory_point[nearest_idx+ff].path_point
                self.ego_speed = self.planned_trajectory.trajectory_point[nearest_idx+ff].v 
            
            #print("v ",self.ego_speed)
            #print("nearest curr point x : ",curr_nearest_point.x , " , y :" , -curr_nearest_point.y , " , yaw: ", - math.degrees(curr_nearest_point.theta))
            #print("next point x : ",nearest_point.x , " , y :" , -nearest_point.y , " , yaw: ", - math.degrees(nearest_point.theta))
            #print("old z ", old_loc.z)
            new_loc = carla.Location(x=nearest_point.x,y=-nearest_point.y,z=old_loc.z)
            new_rot = old_rot
            #print("yaw in carla: ", new_rot.yaw, " theta in apollo: ",current_point.theta)
            new_rot.yaw = - math.degrees(nearest_point.theta)
            transf_end = carla.Transform(new_loc,new_rot)
            self.player.set_transform(transf_end)
            #speed = self.planned_trajectory.trajectory_point[nearest_idx+1].v
            
            
            self.last_trajectory = ADCTrajectory()
            self.last_trajectory.CopyFrom(self.planned_trajectory)
        
        #print((time.time()-self.time)*1000)
        #self.time = time.time()
        # print(self.planned_trajectory.trajectory_point[0])
        #print("callback time: ",(time.time()-t1)*1000)

    def control_callback (self,data):
        self.player.set_simulate_physics(True)

        ##################### get old vehicle control ###################
        old_control = self.player.get_control()

        #################################################################
        vehicle_control = carla.VehicleControl()
        vehicle_control.hand_brake = data.parking_brake
        temp_brake = data.brake/ 100.0
        #if temp_brake > 5:
         #   temp_brake = 4
        vehicle_control.brake = data.brake / 100.0 #/2
        vehicle_control.steer = -1 * data.steering_target / 100.0 
        vehicle_control.throttle =  max(old_control.throttle - 0.01 , data.throttle / 100.0) 
        #vehicle_control.throttle = max(vehicle_control.throttle , 0.2)
        vehicle_control.reverse = data.gear_location == Chassis.GearPosition.GEAR_REVERSE
        #print("vehicle reverse: ", vehicle_control.reverse)
        vehicle_control.gear = 1 if vehicle_control.reverse else -1

        self.player.apply_control(vehicle_control)
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():

    # Open the config file and load the parameters
    params = yaml.safe_load(open("config/bridge_settings.yaml"))
    carla_params = params['carla']
    host = carla_params['host']
    port = carla_params['port']

    # initialize new Carla client
    carla_client = carla.Client(host=host, port=port)
    carla_client.set_timeout(2000)
    carla_world = carla_client.get_world()
    carla_map = carla_world.get_map()
    carla_player = carla_world.get_actors().filter('vehicle.lincoln.mkz*')[0]


    publishing_rate = params['publishing_rate']
    
    apollo_node = ApolloNode(world=carla_world,player=carla_player,params=params)
    
    ct = cyber_timer.Timer(1000/publishing_rate, apollo_node.publish_data, 0)  # 10ms
    ct.start()
    
    apollo_node.node.spin()
    ct.stop()
    #cyber.shutdown()
    


if __name__ == '__main__':

    main()
