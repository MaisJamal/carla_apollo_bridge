#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla gnsss
"""
import math

import carla_common.transforms as trans
from carla_common.euler import euler2quat

#from carla_cyber_bridge.sensor import Sensor

from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose
from modules.drivers.gnss.proto.gnss_status_pb2 import GnssStatus
from modules.drivers.gnss.proto.heading_pb2 import Heading
from modules.localization.proto.gps_pb2 import Gps
from modules.drivers.gnss.proto.ins_pb2 import InsStat

from modules.common.proto.geometry_pb2 import PointENU, Point3D, Quaternion
from modules.localization.proto.pose_pb2 import Pose


from cyber_py import cyber_time


class Gnss():

    """
    Actor implementation details for gnss sensor
    """

    def __init__(self,carla_actor, name,node):
        

        self.uid=carla_actor.id
        self.name=name
        self.parent=carla_actor.parent
        self.node=node
        self.carla_actor=carla_actor
        self.msg_seq_counter = 0
        self.gnss_navsatfix_writer = self.node.create_writer(self.get_topic_prefix() + "/best_pose",
                                           GnssBestPose,
                                           qos_depth=10)
        self.gnss_odometry_writer = self.node.create_writer(self.get_topic_prefix() + "/odometry",
                                           Gps,
                                           qos_depth=10)
        self.gnss_heading_writer = self.node.create_writer(self.get_topic_prefix() + "/heading",
                                           Heading,
                                           qos_depth=10)
        self.gnss_status_writer = self.node.create_writer(self.get_topic_prefix() + "/ins_stat",
                                           InsStat,
                                           qos_depth=10)
        self.carla_actor.listen(self.sensor_data_updated)


    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    def sensor_data_updated(self, carla_gnss_measurement):
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_measurement: carla gnss measurement object
        :type carla_gnss_measurement: carla.GnssMeasurement
        """
        gnss_navsatfix_msg = GnssBestPose()
        gnss_navsatfix_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()#carla_gnss_measurement.timestamp
        gnss_navsatfix_msg.header.sequence_num = self.msg_seq_counter
        gnss_navsatfix_msg.latitude = carla_gnss_measurement.latitude
        gnss_navsatfix_msg.longitude = carla_gnss_measurement.longitude
        gnss_navsatfix_msg.height_msl = carla_gnss_measurement.altitude
        self.gnss_navsatfix_writer.write(gnss_navsatfix_msg)
        


        gnss_odometry_msg = Gps()
        gnss_odometry_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()#carla_gnss_measurement.timestamp
        gnss_odometry_msg.header.sequence_num = self.msg_seq_counter
        gnss_odometry_msg.localization.CopyFrom(trans.carla_transform_to_cyber_pose(self.parent.get_transform()))
        self.gnss_odometry_writer.write(gnss_odometry_msg)

        gnss_heading_msg = Heading()
        gnss_heading_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()#carla_gnss_measurement.timestamp
        gnss_heading_msg.header.sequence_num = self.msg_seq_counter
        gnss_heading_msg.measurement_time = cyber_time.Time.now().to_sec()
        roll, pitch, yaw = trans.carla_rotation_to_RPY(self.carla_actor.get_transform().rotation)
        gnss_heading_msg.heading = yaw 
        self.gnss_heading_writer.write(gnss_heading_msg)

        gnss_status_msg = InsStat()
        gnss_status_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()#carla_gnss_measurement.timestamp
        gnss_status_msg.header.sequence_num = self.msg_seq_counter
        gnss_status_msg.header.module_name = "gnss"
        # gnss_status_msg.solution_completed = True
        # gnss_status_msg.solution_status = 0
        # gnss_status_msg.position_type = 56
        # gnss_status_msg.num_sats = 3
        gnss_status_msg.ins_status = 0
        gnss_status_msg.pos_type = 56
        self.gnss_status_writer.write(gnss_status_msg)
        
        self.msg_seq_counter += 1
