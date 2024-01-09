#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla Lidar sensor
"""


from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud

import numpy as np

from cyber_py import cyber_time


class LidarSensor():

    """
    Actor implementation details for imu sensor
    """

    def __init__(self, carla_actor, name,node):
        

        self.uid=carla_actor.id
        self.name=name
        self.parent=carla_actor.parent
        self.node=node
        self.carla_actor=carla_actor
        self.msg_seq_counter = 0

        self.lidar_writer = self.node.create_writer('/apollo/sensor/lidar128/compensator/PointCloud2', PointCloud)
        self.carla_actor.listen(self.sensor_data_updated)

    def destroy(self):
        super(LidarSensor, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, image):
        """
        Function to transform a received lidar measurement into a Cyber lidar message
        """
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


        self.lidar_writer.write(lidar_msg)
