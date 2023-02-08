

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



# ==============================================================================
# ------- get localization message ---------------------------------------------
# ==============================================================================

def get_localization_msg(carla_actor_player):

    transform = carla_actor_player.get_transform()
    linear_vel = carla_actor_player.get_velocity()
    angular_vel = carla_actor_player.get_angular_velocity()
    accel = carla_actor_player.get_acceleration()
    heading = -math.radians(transform.rotation.yaw)

    localization_msg = LocalizationEstimate()
    
    localization_msg.measurement_time = cyber_time.Time.now().to_sec()
    localization_msg.header.frame_id = 'localization'
    localization_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()

    ####################### Carla transform.location to Apollo Pose ################################
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
    #self.transform_writer.write(transform_msg)


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
# ------- TO DO ----------------------------------------------------------------
# ==============================================================================


def get_obstacles_msg(world,player) : 
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
        obs_transform = obs.get_transform()
        obs_velocity = obs.get_velocity()
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
        if isinstance(obs, carla.Walker):
            obstacle.type = Type.PEDESTRIAN
        else:
            obstacle.type = Type.VEHICLE

        obstacle.length = box.extent.x * 2 
        obstacle.width = box.extent.y * 2 
        obstacle.height = box.extent.z * 2 

    return obstacles


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
