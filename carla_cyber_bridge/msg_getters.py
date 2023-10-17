

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

import numpy as np
import math
import cv2

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
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection


from modules.drivers.proto.sensor_image_pb2 import CompressedImage

from carla_common.euler import euler2mat, quat2euler, euler2quat
from scipy.spatial.transform import Rotation as R



# ==============================================================================
# -- global variables ----------------------------------------------------------
# ==============================================================================


class Type:    # obstacle type in proto in apollo v7.0.0 has modifications of the one in apollo v5.0.0
    UNKNOWN = 0
    UNKNOWN_MOVABLE = 1
    UNKNOWN_UNMOVABLE = 2
    PEDESTRIAN = 3   # Pedestrian, usually determined by moving behavior.
    BICYCLE = 4      # bike, motor bike
    VEHICLE = 5      # Passenger car or truck.


# IDs mapping of traffic signals, from Carla Town 01 to Apollo Town 01

tl_dict = {(323, 4):"signal0"    ,(324, 333):"signal1"  ,(349, 324):"signal2"  ,(77, 4):"signal3"     ,(167, -4):"signal4"
           ,(160, 12):"signal5"  ,(143, 4):"signal6"    ,(341, 209):"signal7"  ,(323, 202):"signal8"  ,(331, 184):"signal9"
           ,(341, 143):"signal10",(321, 136):"signal11" ,(331, 118):"signal12" ,(341, 69):"signal13"  ,(348, -4):"signal14" 
           ,(341, 12):"signal15" ,(321, 62):"signal16"  ,(331, 44):"signal17"  ,(144, 62):"signal18"  ,(151, 45):"signal19" 
           ,(168, 53):"signal20" ,(95, 12):"signal21"   ,(332, 316):"signal22" ,(102, -4):"signal23"  ,(75, 333):"signal24"
           ,(85, 316):"signal25" ,(106, 324):"signal26" ,(85, 185):"signal27"  ,(102, 192):"signal28" ,(94, 209):"signal29"
           ,(85, 119):"signal30" ,(102, 126):"signal31" ,(94, 144):"signal32"  ,(85, 45):"signal33"   ,(102, 52):"signal34" 
           ,(94, 70):"signal35" 
            }

def Euclidean_distance(v1_trans,v2_trans):
    x1 = v1_trans.x
    y1 = v1_trans.y
    x2 = v2_trans.x
    y2 = v2_trans.y
    p = math.pow(x1-x2 , 2) + math.pow(y1-y2 , 2)
    return math.sqrt(p)



# ==============================================================================
# ------- get localization message ---------------------------------------------
# ==============================================================================

def n2b(x_radius, y_radius, z_radius, b):
    x_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(x_radius), -np.sin(x_radius)],
            [0, np.sin(x_radius), np.cos(x_radius)]])
    y_matrix = np.array([
            [np.cos(y_radius), 0, np.sin(y_radius)],
            [0, 1, 0],
            [-np.sin(y_radius), 0, np.cos(y_radius)]])
    z_matrix = np.array([
            [np.cos(z_radius), -np.sin(z_radius), 0],
            [np.sin(z_radius), np.cos(z_radius), 0],
            [0, 0, 1]])
    n = np.matrix(np.array(b)) * np.matrix(x_matrix) * np.matrix(y_matrix) * np.matrix(z_matrix)
    return n

def get_apollo_pos_shifted(pos, heading, shift):
    x = pos.x
    y = -pos.y
    z = pos.z
    return [x - shift * math.cos(heading), y - shift * math.sin(heading), z]





def get_localization_msg(carla_actor_player):
    carla_transform = carla_actor_player.get_transform()
    carla_linear_vel = carla_actor_player.get_velocity()
    carla_angular_vel = carla_actor_player.get_angular_velocity()
    carla_acc = carla_actor_player.get_acceleration()

    heading = -math.radians(carla_transform.rotation.yaw)

    localization_msg = LocalizationEstimate()
    
    localization_msg.measurement_time = cyber_time.Time.now().to_sec()
    localization_msg.header.frame_id = 'localization'
    localization_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()

    ####################### Carla transform.location to Apollo Pose ################################
    x_sh, y_sh, z_sh = get_apollo_pos_shifted(carla_transform.location, heading, 1.355)
    localization_msg.pose.position.x = x_sh
    localization_msg.pose.position.y = y_sh
    localization_msg.pose.position.z = z_sh

    #######################################################################################
    ####################### Carla transform.rotation to Apollo Quaternion #################

    roll = math.radians(carla_transform.rotation.roll)
    pitch = -math.radians(carla_transform.rotation.pitch)
    yaw = (((-math.radians(carla_transform.rotation.yaw + 90) % 2.*math.pi) + 2.*math.pi) % 2.*math.pi)

    quat = euler2quat(roll, pitch, yaw)  # w , x, y, z
    localization_msg.pose.orientation.qw = quat[0]
    localization_msg.pose.orientation.qx = quat[1]
    localization_msg.pose.orientation.qy = quat[2]
    localization_msg.pose.orientation.qz = quat[3]

    localization_msg.pose.euler_angles.x = 0 #roll_apollo           ## for testing
    localization_msg.pose.euler_angles.y = 0 #pitch_apollo         ## for testing
    localization_msg.pose.euler_angles.z = yaw    ## for testing

    #######################################################################################
    ####################### Carla velocity to Apollo linear and angular velocity ##########
    ## linear velocity and rotation are needed
    
    localization_msg.pose.linear_velocity.x = carla_linear_vel.y 
    localization_msg.pose.linear_velocity.y = carla_linear_vel.x
    localization_msg.pose.linear_velocity.z = carla_linear_vel.z

    localization_msg.pose.angular_velocity.x = carla_angular_vel.x #  math.radians(angular_vel.x)
    localization_msg.pose.angular_velocity.y = -carla_angular_vel.y # -math.radians(angular_vel.y)
    localization_msg.pose.angular_velocity.z = -carla_angular_vel.z # -math.radians( angular_vel.z)

    linear_a_x = carla_acc.x    ## for testing- accel.y   ## for testing
    linear_a_y = -carla_acc.y   ## for testing- accel.x   ## for testing
    linear_a_z = carla_acc.z   ## for testing

    localization_msg.pose.linear_acceleration.x = linear_a_x # linear_a_x        ## for testing  / it was accel.x
    localization_msg.pose.linear_acceleration.y = linear_a_y # linear_a_y       ## for testing  / it was -accel.y
    localization_msg.pose.linear_acceleration.z = linear_a_z # linear_a_z  ## for testing

    ################ linear acceleration in vehicle coordination system in Apollo #################

    enu_accel_velocity = n2b(pitch, roll, yaw, np.array([linear_a_x, linear_a_y, linear_a_z]))
    localization_msg.pose.linear_acceleration_vrf.x = enu_accel_velocity[0, 0]
    localization_msg.pose.linear_acceleration_vrf.y = enu_accel_velocity[0, 1]
    localization_msg.pose.linear_acceleration_vrf.z = enu_accel_velocity[0, 2]

    enu_angular_velocity = n2b(pitch, roll, yaw, np.array([carla_angular_vel.x,
                                                                -carla_angular_vel.y,
                                                                -carla_angular_vel.z]))
    localization_msg.pose.angular_velocity_vrf.x = enu_angular_velocity[0, 0]
    localization_msg.pose.angular_velocity_vrf.y = enu_angular_velocity[0, 1]
    localization_msg.pose.angular_velocity_vrf.z = enu_angular_velocity[0, 2]
    localization_msg.pose.heading = heading

    return localization_msg

# ==============================================================================
# ------- get chassis message --------------------------------------------------
# ==============================================================================

def get_chassis_msg(carla_actor_player,planning_flag,speed):
    v = carla_actor_player.get_velocity()
    c = carla_actor_player.get_control()
    if not planning_flag:
        speed_abs = math.sqrt(v.x**2 + v.y**2 + v.z**2)   # in m/s
    else :
        speed_abs = speed
    chassis_msg = Chassis()
    chassis_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    #chassis_msg.header.sequence_num = self.chassis_sequence_num
    #self.chassis_sequence_num += 1
    chassis_msg.engine_started = True
    #print("speed: ", speed_abs)
    chassis_msg.speed_mps = speed_abs
    chassis_msg.throttle_percentage = c.throttle * 100.0 
    chassis_msg.brake_percentage = c.brake * 100.0 
    chassis_msg.steering_percentage = -1* c.steer * 100.0
    chassis_msg.parking_brake = c.hand_brake
    chassis_msg.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
    if c.reverse :
        chassis_msg.gear_location = Chassis.GearPosition.GEAR_REVERSE
        #print("gear reverse")
    else :
        chassis_msg.gear_location = Chassis.GearPosition.GEAR_DRIVE
        #print("gear drive")


    return chassis_msg



# ==============================================================================
# ------- get obstacles message ------------------------------------------------
# ------- ----- ----------------------------------------------------------------
# ==============================================================================


def get_obstacles_msg(world,player,detection_radius) : 
    other_vehicles_num = 0
    walkers_num = 0
    hero_num = 0
    pedestrians = []
    all_obstacles = []
    #print("actors: ",world.get_actors)

    for actor in world.get_actors():
        #print("act ",actor)
        if actor.attributes.get('role_name') == 'hero':
            hero_num += 1
            continue
        else:
            distance = Euclidean_distance(actor.get_transform().location,player.get_transform().location)
            if distance <= detection_radius:
                if isinstance(actor, carla.Vehicle) and actor.attributes.get('role_name') != 'hero':
                    other_vehicles_num +=1
                    all_obstacles.append(actor)
                    
                elif isinstance(actor, carla.Walker):
                    walkers_num +=1
                    pedestrians.append(actor)
                    all_obstacles.append(actor)

    #print("hereos: ",hero_num)      
    #print("walkers: ", walkers_num)
    #print("obstacle vehicles: ", other_vehicles_num)        

    obstacles = PerceptionObstacles()
    obstacles.header.timestamp_sec = cyber_time.Time.now().to_sec()
    obstacles.header.module_name = 'perception_obstacle'
    for obs in all_obstacles:
        #print("car id ",car.id)
        is_bike = False
        obs_transform = obs.get_transform()
        obs_velocity = obs.get_velocity()
        if obs.attributes.get('number_of_wheels') == str(2):
            is_bike = True
        box = obs.bounding_box
        #print(box.location)         # Location relative to the vehicle.
        #print(box.extent) 
        
        obstacle = obstacles.perception_obstacle.add()
        obstacle.id = obs.id
        obstacle.theta = -math.radians(obs_transform.rotation.yaw)   # in radian
        obstacle.position.x = obs_transform.location.x
        obstacle.position.y = - obs_transform.location.y
        obstacle.position.z = 0

        obstacle.velocity.x = obs_velocity.x
        obstacle.velocity.y = - obs_velocity.y
        obstacle.velocity.z = 0

        obstacle.length = box.extent.x * 2 
        obstacle.width = box.extent.y * 2 
        obstacle.height = box.extent.z * 2 

        ### TODO add bike check/BICYCLE type
        if isinstance(obs, carla.Walker):
            obstacle.type = Type.PEDESTRIAN
        else:
            if is_bike:
                obstacle.type = Type.BICYCLE
                if box.extent.y < 0.1 :
                    obstacle.width = 0.75
                if (box.extent.z * 2 ) > 2 :
                    obstacle.height = 1.9

            #elif obs_velocity.x == 0 and obs_velocity.y ==0:
            #    obstacle.type = Type.UNKNOWN_UNMOVABLE
            
            else:
                obstacle.type = Type.VEHICLE

        

    return obstacles


# ==============================================================================
# ------- get traffic lights message ------------------------------------------------
# ------- ----- ----------------------------------------------------------------
# ==============================================================================

def get_tr_lights_msg(world):
    all_traffic_lights = world.get_actors().filter('traffic.traffic_light*')

    #print("number of traffic lights: ", len(all_traffic_lights) )
    lights = TrafficLightDetection()
    lights.header.timestamp_sec = cyber_time.Time.now().to_sec()
    lights.header.module_name = 'traffic_light'

    for tl in all_traffic_lights:
        #print(tl.id, tl.get_state(), tl.get_location().x, tl.get_location().y)
        #print((int(tl.get_location().x),int(tl.get_location().y)) )
        if (int(tl.get_location().x),int(tl.get_location().y)) in tl_dict:
            #print(tl.id)
            ###############################
            ######### RED = 1 #############
            ######### YELLOW = 2 ##########
            ######### GREEN = 3 ###########
            ###############################
            if tl.get_state() == carla.libcarla.TrafficLightState.Red:
                apollo_color = 1
            elif tl.get_state() == carla.libcarla.TrafficLightState.Yellow:
                apollo_color = 2
            elif tl.get_state() == carla.libcarla.TrafficLightState.Green:
                apollo_color = 3

            traffic_light = lights.traffic_light.add()

            traffic_light.color = apollo_color
            traffic_light.id = tl_dict[(int(tl.get_location().x),int(tl.get_location().y))]
            traffic_light.confidence = 1.000000
            traffic_light.tracking_time = 1.0000000

    return lights

# ==============================================================================
# ------- get camera message ---------------------------------------------------
# ------- TO DO ----------------------------------------------------------------
# ==============================================================================

def get_camera_msg(image):
   
    image_data_array = np.ndarray(shape=(720, 1280, 4), dtype=np.uint8, buffer=image.raw_data)
    cyber_img = CompressedImage()
    
    #cyber_img.frame_id = cyber_img.header.frame_id
    cyber_img.format = 'jpeg'
    #cyber_img.measurement_time = cyber_img.header.timestamp_sec
    cyber_img.data = cv2.imencode('.jpg', image_data_array)[1].tostring()
    return cyber_img
