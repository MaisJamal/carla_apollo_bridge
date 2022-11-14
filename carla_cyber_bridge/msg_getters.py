

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

import numpy as np
import math


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

from carla_common.euler import euler2mat, quat2euler, euler2quat
from scipy.spatial.transform import Rotation as R


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