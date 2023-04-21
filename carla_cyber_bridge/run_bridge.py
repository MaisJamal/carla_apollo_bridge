#!/usr/bin/env python

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import yaml
import carla
import math
import time


from cyber_py import cyber, cyber_time , cyber_timer

from gnss import Gnss
from imu import ImuSensor

from msg_getters import get_chassis_msg ,get_localization_msg , get_obstacles_msg ,get_camera_msg , get_tr_lights_msg

from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.transform.proto.transform_pb2 import TransformStamped, TransformStampeds,Transform
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.drivers.gnss.proto.imu_pb2 import Imu
from modules.localization.proto.gps_pb2 import Gps
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.drivers.proto.sensor_image_pb2 import CompressedImage

# ==============================================================================
# -- global variables ----------------------------------------------------------
# ==============================================================================


class Scenarios:    # scenario_type in proto in apollo v7.0.0 has modifications of the one in apollo v5.0.0
    LANE_FOLLOW = 0  
    BARE_INTERSECTION_UNPROTECTED = 2
    STOP_SIGN_PROTECTED = 3
    STOP_SIGN_UNPROTECTED = 4
    TRAFFIC_LIGHT_PROTECTED = 5
    TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN = 6
    TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN = 7
    YIELD_SIGN = 8

    PULL_OVER = 9
    VALET_PARKING = 10

    EMERGENCY_PULL_OVER = 11
    EMERGENCY_STOP = 1

    NARROW_STREET_U_TURN = 13
    PARK_AND_GO = 14

    LEARNING_MODEL_SAMPLE = 15
    DEADEND_TURNAROUND = 16

class Stage:
    VALET_PARKING_APPROACHING_PARKING_SPOT = 700
    VALET_PARKING_PARKING = 0



# ==============================================================================
# -- Apollo Node ---------------------------------------------------------------
# ==============================================================================


class ApolloNode:

    def __init__(self,world,player,sensors,params):
        cyber.init()
        self.sim_world = world
        self.player = player
        self.sensors = sensors
        self.node = cyber.Node("bridge_node")
        self.params = params
        self.detection_radius = 0
        self.msg_seq_counter = 0
        self.ego_speed = 0
        self.planning_flag = 0
        self.time = time.time()
        self.planned_trajectory = None
        self.last_trajectory = None
        
        first_trans = self.player.get_transform()
        self.last_x = first_trans.location.x
        self.last_y = - first_trans.location.y
        self.last_theta = - math.radians(first_trans.rotation.yaw)
        self.last_image = None

        

        if self.params['publish_localization_chassis_msgs']:
            #self.location_writer = self.node.create_writer(params['localization_channel'], LocalizationEstimate)
            self.chassis_writer = self.node.create_writer(params['chassis_channel'], Chassis)

        if self.params['publish_gnss']:
            gnss_actors = world.get_actors().filter('sensor.other.gnss*')
            if not gnss_actors:
                print("No Gnss sensor associated with the vehicle...")
            else:
                self.sensors['gnss'] = Gnss(gnss_actors[0],"gnss",self.node)
        
        if self.params['publish_imu']:
            imu_actors = world.get_actors().filter('sensor.other.imu*')
            if not imu_actors:
                print("No Imu sensor associated with the vehicle...")
            else:
                self.sensors['imu'] = ImuSensor(imu_actors[0],"imu",self.node)
        
        if self.params['publish_camera_msg']:
            self.camera_writer = self.node.create_writer(params['camera_channel'] , CompressedImage)

        if self.params['publish_obstacles_ground_truth']:
            self.obstacles_writer = self.node.create_writer(params['perception_channel'] , PerceptionObstacles)
            self.detection_radius = params['detection_radius']

        if self.params['publish_traffic_light_gt']:
            self.traffic_lights_writer = self.node.create_writer(params['traffic_light_channel'] , TrafficLightDetection)
        
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

            #localization_msg = get_localization_msg(self.player)
            #localization_msg.header.sequence_num = self.msg_seq_counter
            #self.location_writer.write(localization_msg)

        if self.params['publish_camera_msg']:
            camera_msg = get_camera_msg(self.last_image)
            camera_msg.header.sequence_num = self.msg_seq_counter
            #camera_msg.measurement_time
            self.camera_writer.write(camera_msg) 
        
        if self.params['publish_obstacles_ground_truth']:
            obstacles = get_obstacles_msg(self.sim_world,self.player,self.detection_radius)
            obstacles.header.sequence_num = self.msg_seq_counter
            self.obstacles_writer.write(obstacles)

        if self.params['publish_traffic_light_gt']:
            lights = get_tr_lights_msg(self.sim_world)
            lights.header.sequence_num = self.msg_seq_counter
            self.traffic_lights_writer.write(lights)

        self.msg_seq_counter += 1
    
    def planning_callback(self,msg):
        t1 = time.time()
       # print("time from last callback: ", t1-self.time)
        self.time = t1
        self.planned_trajectory = ADCTrajectory()
        self.planned_trajectory.CopyFrom(msg)
        
        
        #print("sc type ",self.planned_trajectory.debug.planning_data.scenario.scenario_type)
        #print("stage type ",self.planned_trajectory.debug.planning_data.scenario.stage_type)
        #print("scenario ",self.planned_trajectory.debug.planning_data.scenario)
        is_in_parking = False
        if self.planned_trajectory.debug.planning_data.scenario.scenario_type == Scenarios.VALET_PARKING :#and self.planned_trajectory.debug.planning_data.scenario.stage_type == Stage.VALET_PARKING_PARKING:  
            print("scenario is valet parking ......")
            is_in_parking = True

        if not is_in_parking:
            ff = 5
        else:
            ff = 1 
        #print("ff is ",ff)
        self.player.set_simulate_physics(True)

        if len(self.planned_trajectory.trajectory_point)<1:
            if not self.last_trajectory :
                print("Received trajectory is empty.")
            elif not is_in_parking:
                #print("Suddenly received trajectory is empty.")
                old_trans = self.player.get_transform()
                old_loc = old_trans.location
                old_rot = old_trans.rotation

                dx_nearest_to_vehicle = self.last_trajectory.trajectory_point[0].path_point.x - self.last_x
                dy_nearest_to_vehicle = self.last_trajectory.trajectory_point[0].path_point.y - self.last_y
                nearest_dist = math.sqrt(dx_nearest_to_vehicle * dx_nearest_to_vehicle + dy_nearest_to_vehicle * dy_nearest_to_vehicle)
                nearest_idx = 0

                i=0
                for tp in self.last_trajectory.trajectory_point:
                    dx_current_to_vehicle = self.last_trajectory.trajectory_point[i].path_point.x - self.last_x 
                    dy_current_to_vehicle = self.last_trajectory.trajectory_point[i].path_point.y - self.last_y
                    current_dist_to_vehicle = math.sqrt(dx_current_to_vehicle * dx_current_to_vehicle 
                                        + dy_current_to_vehicle * dy_current_to_vehicle)
                    if current_dist_to_vehicle < nearest_dist :
                        nearest_dist = current_dist_to_vehicle
                        nearest_idx = i
                
                    i+=1
                
                nearest_point = self.last_trajectory.trajectory_point[min(nearest_idx+ff,i-1)].path_point
                self.ego_speed = self.last_trajectory.trajectory_point[min(nearest_idx+ff,i-1)].v
                shift = 1.355
                heading = nearest_point.theta
                shifted_x = nearest_point.x + shift * math.cos(heading)
                shifted_y = - (nearest_point.y + shift * math.sin(heading))
            
                new_loc = carla.Location(x=shifted_x,y=shifted_y,z=old_loc.z)
                new_rot = old_rot
                new_rot.yaw = - math.degrees(nearest_point.theta)
                transf_end = carla.Transform(new_loc,new_rot)

                self.player.set_transform(transf_end)

                self.last_x = nearest_point.x
                self.last_y = nearest_point.y
                self.last_theta = nearest_point.theta
                

        else:
            old_trans = self.player.get_transform()
            old_loc = old_trans.location
            old_rot = old_trans.rotation
            
            
            dx_nearest_to_vehicle = self.planned_trajectory.trajectory_point[0].path_point.x - self.last_x 
            dy_nearest_to_vehicle = self.planned_trajectory.trajectory_point[0].path_point.y - self.last_y
            nearest_dist = math.sqrt(dx_nearest_to_vehicle * dx_nearest_to_vehicle + dy_nearest_to_vehicle * dy_nearest_to_vehicle)
            nearest_idx = 0

            i=0
            tt1 = time.time()
            for tp in self.planned_trajectory.trajectory_point:
                dx_current_to_vehicle = self.planned_trajectory.trajectory_point[i].path_point.x - self.last_x
                dy_current_to_vehicle = self.planned_trajectory.trajectory_point[i].path_point.y - self.last_y
                current_dist_to_vehicle = math.sqrt(dx_current_to_vehicle * dx_current_to_vehicle 
                                    + dy_current_to_vehicle * dy_current_to_vehicle)
                if current_dist_to_vehicle < nearest_dist :
                    nearest_dist = current_dist_to_vehicle
                    nearest_idx = i
                #print("i: ", i , " x: ",self.planned_trajectory.trajectory_point[i].path_point.x," y: ", self.planned_trajectory.trajectory_point[i].path_point.y)
                i+=1
            
            
            nearest_point = self.planned_trajectory.trajectory_point[min(nearest_idx+ff,i-1)].path_point
            self.ego_speed = self.planned_trajectory.trajectory_point[min(nearest_idx+ff,i-1)].v
            

            shift = 1.355
            heading = nearest_point.theta
            shifted_x = nearest_point.x + shift * math.cos(heading)
            shifted_y = - (nearest_point.y + shift * math.sin(heading))
            #print("shifted x: ",shifted_x," , shifted y : ",shifted_y)

            new_loc = carla.Location(x=shifted_x,y=shifted_y,z=old_loc.z)
            new_rot = old_rot
            new_rot.yaw = - math.degrees(nearest_point.theta)

            transf_end = carla.Transform(new_loc,new_rot)

            if not is_in_parking or nearest_dist<0.6: 
                self.player.set_transform(transf_end)
            
                self.last_x = nearest_point.x
                self.last_y = nearest_point.y
                self.last_theta = nearest_point.theta
            
            
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


    def camera_callback(self,image):
        image.convert(carla.ColorConverter.Raw)
        self.last_image = image
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():

    # Open the config file and load the parameters
    params = yaml.safe_load(open("config/bridge_settings.yaml"))
    carla_params = params['carla']
    host = carla_params['host']
    port = carla_params['port']

    ########### initialize new Carla client ##########
    carla_client = carla.Client(host=host, port=port)
    carla_client.set_timeout(2000)
    carla_world = carla_client.get_world()
    carla_map = carla_world.get_map()

    for actor in carla_world.get_actors():
        if actor.attributes.get('role_name') == 'hero':
            carla_player = actor
    #carla_player = carla_world.get_actors().filter('vehicle.lincoln.mkz*')[0]

    ########### get sensor actors ##########
    sensors = {}
    camera = carla_world.get_actors().filter('sensor.camera.rgb')[0]
    if camera:
        sensors['camera'] = camera

    publishing_rate = params['publishing_rate']
    
    apollo_node = ApolloNode(world=carla_world,player=carla_player,sensors = sensors,params=params)
    ###############################
    camera.listen(apollo_node.camera_callback)
    ######################################
    ct = cyber_timer.Timer(1000/publishing_rate, apollo_node.publish_data, 0)  # 10ms
    ct.start()
    
    apollo_node.node.spin()
    ct.stop()
    #cyber.shutdown()
    


if __name__ == '__main__':

    main()
