#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla imu sensor
"""


from modules.drivers.gnss.proto.imu_pb2 import Imu
from modules.localization.proto.imu_pb2 import CorrectedImu

from cyber_py import cyber_time


class ImuSensor():

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

        # self.imu_writer = node.new_writer(self.get_topic_prefix(), Imu, qos_depth=10)
        self.imu_writer = self.node.create_writer('/apollo/sensor/gnss/corrected_imu', CorrectedImu, qos_depth=20)
        self.carla_actor.listen(self.sensor_data_updated)

    def destroy(self):
        super(ImuSensor, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_imu_measurement):
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param carla_imu_measurement: carla imu measurement object
        :type carla_imu_measurement: carla.IMUMeasurement
        """
        imu_msg = CorrectedImu()
        imu_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()#carla_gnss_measurement.timestamp

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        imu_msg.imu.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.imu.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.imu.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        acc = carla_imu_measurement.accelerometer.x
        if acc > 2.0:
            acc = 2.0
        elif acc < -2.0:
            acc = -2.0
        imu_msg.imu.linear_acceleration.x = -carla_imu_measurement.accelerometer.y
        imu_msg.imu.linear_acceleration.y = acc #carla_imu_measurement.accelerometer.x
        imu_msg.imu.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        imu_msg.imu.euler_angles.x = carla_imu_measurement.transform.rotation.roll / 180 * 3.14
        imu_msg.imu.euler_angles.y = carla_imu_measurement.transform.rotation.pitch / 180 * 3.14
        imu_msg.imu.euler_angles.z = carla_imu_measurement.transform.rotation.yaw / 180 * 3.14

        self.imu_writer.write(imu_msg)
