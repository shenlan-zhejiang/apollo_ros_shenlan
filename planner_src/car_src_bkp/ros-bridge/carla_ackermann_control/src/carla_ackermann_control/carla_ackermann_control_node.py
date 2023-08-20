#! /usr/bin/python3

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Control Carla ego vehicle by using AckermannDrive messages
"""

import sys

import rospkg
import carla

import numpy
# import torch
from simple_pid import PID  # pylint: disable=import-error,wrong-import-order

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.exceptions import *
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ackermann_control import carla_control_physics as phys

from ackermann_msgs.msg import AckermannDrive  # pylint: disable=import-error,wrong-import-order
from std_msgs.msg import Header # pylint: disable=wrong-import-order
from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_msgs.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error

ROS_VERSION = roscomp.get_ros_version()

if ROS_VERSION == 1:
    from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig # pylint: disable=no-name-in-module,import-error,ungrouped-imports
    from dynamic_reconfigure.server import Server # pylint: disable=no-name-in-module,import-error
if ROS_VERSION == 2:
    from rcl_interfaces.msg import SetParametersResult


class CarlaAckermannControl(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        super(CarlaAckermannControl, self).__init__("carla_ackermann_control")


        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.vehicle_info = None
        self.vehicle_actor = None
        self.world = None

        self.connect_to_carla()

        self.ackermann_control = carla.VehicleAckermannControl()
        self.ackermann_controller_settings = carla.AckermannControllerSettings()

        if ROS_VERSION == 1:
            self.reconfigure_server = Server(
                EgoVehicleControlParameterConfig,
                namespace="",
                callback=self.reconfigure_pid_parameters,
            )
        if ROS_VERSION == 2:
            self. add_on_set_parameters_callback(self.reconfigure_pid_parameters)

        self.control_loop_period = self.get_param("control_loop_period", 0.01)
        self.last_ackermann_msg_received_sec = self.get_time()
        self.vehicle_status = CarlaEgoVehicleStatus()


        # control info
        self.info = EgoVehicleControlInfo()

        # set initial maximum values
        self.vehicle_info_updated(self.vehicle_info)

        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.

        # current values
        self.info.current.time_sec = self.get_time()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.

        # control values
        self.accel_by_speed_pid = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.

        # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = False

        # ackermann drive commands
        self.control_subscriber = self.new_subscription(
            AckermannDrive,
            "/carla/" + self.role_name + "/ackermann_cmd",
            self.ackermann_command_callback,
            qos_profile=10
        )

        # report controller info
        self.control_info_publisher = self.new_publisher(
            EgoVehicleControlInfo,
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            qos_profile=1)

    if ROS_VERSION == 1:

        def reconfigure_pid_parameters(self, ego_vehicle_control_parameter, _level):
            """
            Callback for dynamic reconfigure call to set the PID parameters

            :param ego_vehicle_control_parameter:
            :type ego_vehicle_control_parameter: \
                carla_ackermann_control.cfg.EgoVehicleControlParameterConfig

            """
            self.loginfo("Reconfigure Request:  "
                         "speed ({speed_Kp}, {speed_Ki}, {speed_Kd}), "
                         "accel ({accel_Kp}, {accel_Ki}, {accel_Kd})"
                         "".format(**ego_vehicle_control_parameter))

            self.ackermann_controller_settings.speed_kp = ego_vehicle_control_parameter['speed_Kp']
            self.ackermann_controller_settings.speed_ki = ego_vehicle_control_parameter['speed_Ki']
            self.ackermann_controller_settings.speed_kd = ego_vehicle_control_parameter['speed_Kd']
            self.ackermann_controller_settings.accel_kp = ego_vehicle_control_parameter['accel_Kp']
            self.ackermann_controller_settings.accel_ki = ego_vehicle_control_parameter['accel_Ki']
            self.ackermann_controller_settings.accel_kd = ego_vehicle_control_parameter['accel_Kd']

            if self.vehicle_actor:
                self.vehicle_actor.apply_ackermann_controller_settings(self.ackermann_controller_settings)

            return ego_vehicle_control_parameter

    # if ROS_VERSION == 2:
    #
    #     def reconfigure_pid_parameters(self, params):  # pylint: disable=function-redefined
    #         """Check and update the node's parameters."""
    #         param_values = {p.name: p.value for p in params}
    #
    #         pid_param_names = {
    #             "speed_Kp",
    #             "speed_Ki",
    #             "speed_Kd",
    #             "accel_Kp",
    #             "accel_Ki",
    #             "accel_Kd",
    #         }
    #         common_names = pid_param_names.intersection(param_values.keys())
    #         if not common_names:
    #             # No work do to
    #             return SetParametersResult(successful=True)
    #
    #         if any(p.value is None for p in params):
    #             return SetParametersResult(
    #                 successful=False, reason="Parameter must have a value assigned"
    #             )
    #
    #         self.speed_controller.tunings = (
    #             param_values.get("speed_Kp", self.speed_controller.Kp),
    #             param_values.get("speed_Ki", self.speed_controller.Ki),
    #             param_values.get("speed_Kd", self.speed_controller.Kd),
    #         )
    #         self.accel_controller.tunings = (
    #             param_values.get("accel_Kp", self.accel_controller.Kp),
    #             param_values.get("accel_Ki", self.accel_controller.Ki),
    #             param_values.get("accel_Kd", self.accel_controller.Kd),
    #         )
    #
    #         self.loginfo(
    #             "Reconfigure Request:  speed ({}, {}, {}), accel ({}, {}, {})".format(
    #                 self.speed_controller.tunings[0],
    #                 self.speed_controller.tunings[1],
    #                 self.speed_controller.tunings[2],
    #                 self.accel_controller.tunings[0],
    #                 self.accel_controller.tunings[1],
    #                 self.accel_controller.tunings[2]
    #             )
    #         )
    #
    #         return SetParametersResult(successful=True)

    def connect_to_carla(self):

        self.loginfo("Waiting for CARLA vehicle info '/carla/{}/vehicle_info'".format(self.role_name))
        try:
            self.vehicle_info = self.wait_for_message(
                                "/carla/" + self.role_name + "/vehicle_info",
                                CarlaEgoVehicleInfo,
                                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for vehicle info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
            self.vehicle_actor = self.world.get_actor(self.vehicle_info.id)
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")

    def get_msg_header(self):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        header.frame_id = "map"
        header.stamp = roscomp.ros_timestamp(sec=self.get_time(), from_sec=True)
        return header

    def vehicle_info_updated(self, vehicle_info):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # set target values
        self.vehicle_info = vehicle_info

        # calculate restrictions
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.vehicle_info)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(
            self.vehicle_info)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(
            self.vehicle_info)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(
            self.vehicle_info)
        self.info.restrictions.min_accel = self.get_param('min_accel', 1.)
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def ackermann_command_callback(self, ros_ackermann_drive):
        self.last_ackermann_msg_received_sec = self.get_time()

        self.ackermann_control.steer = self.info.target.steering_angle = -ros_ackermann_drive.steering_angle
        self.ackermann_control.steer /= 0.7        
        self.control_info_publisher.publish(self.info) 
        self.ackermann_control.speed = self.info.target.speed = ros_ackermann_drive.speed
        self.ackermann_control.acceleration = self.info.target.accel = ros_ackermann_drive.acceleration
        self.vehicle_actor.apply_ackermann_control(self.ackermann_control)



    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """
        # finally clip the final control output (should actually never happen)
        self.info.output.brake = numpy.clip(
            self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(
            self.info.output.throttle, 0., 1.)

    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        self.control_info_publisher.publish(self.info)

    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = self.get_time()
        delta_time = current_time_sec - self.info.current.time_sec
        current_speed = self.vehicle_status.velocity
        if delta_time > 0:
            delta_speed = current_speed - self.info.current.speed
            current_accel = delta_speed / delta_time
            # average filter
            self.info.current.accel = (self.info.current.accel * 4 + current_accel) / 5
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def run(self):
        """
        Control loop
        :return:
        """
        self.spin()


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("carla_ackermann_control", args=args)

    try:
        controller = CarlaAckermannControl()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
