#!/usr/bin/python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
provide functions to control actors
"""

import carla
import ast

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_common.transforms import ros_pose_to_carla_transform

from ackermann_msgs.msg import AckermannDrive
from carla_ackermann_msgs.msg import EgoVehicleControlInfo 


import ros_compatibility as roscomp

ROS_VERSION = roscomp.get_ros_version()

if ROS_VERSION == 1:
    from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig # pylint: disable=no-name-in-module,import-error,ungrouped-imports
    from dynamic_reconfigure.server import Server # pylint: disable=no-name-in-module,import-error
if ROS_VERSION == 2:
    from rcl_interfaces.msg import SetParametersResult

class AckermannControl(PseudoActor):

    """
    provide functions to control actors
    """

    def __init__(self, uid, name, parent, node, attributes=None):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identifying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(AckermannControl, self).__init__(uid=uid,
                                               name=name,
                                               parent=parent,
                                               node=node)

        self.ackermann_control = carla.VehicleAckermannControl()
        self.ackermann_controller_settings = carla.AckermannControllerSettings()

        if attributes:
            self.load_parameters(attributes)

        if self.parent.carla_actor.is_alive:
            self.parent.carla_actor.apply_ackermann_controller_settings(self.ackermann_controller_settings)

        # if ROS_VERSION == 1:
        #     self.reconfigure_server = Server(
        #         EgoVehicleControlParameterConfig,
        #         namespace=self.get_topic_prefix(),
        #         callback=self.reconfigure_pid_parameters)
        # TODO: add ros 2 support
        # if ROS_VERSION == 2:
        #     self.add_on_set_parameters_callback(self.reconfigure_pid_parameters)


        self.ackermann_cmd_subscriber = self.node.new_subscription(
            AckermannDrive,
            self.parent.get_topic_prefix() + "/ackermann_cmd",
            self.ackermann_cmd_callback,
            qos_profile=10
        )

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        self.node.destroy_subscription(self.ackermann_cmd_subscriber)
        super(AckermannControl, self).destroy()

    def load_parameters(self, attributes):
        try:
            for attribute in attributes:
                if attribute.key == "pid_params":
                    # print(type(attribute.value), attribute.value)
                    pid_params = ast.literal_eval(attribute.value)
                    self.ackermann_controller_settings.speed_kp = pid_params['speed_Kp']
                    self.ackermann_controller_settings.speed_ki = pid_params['speed_Ki']
                    self.ackermann_controller_settings.speed_kd = pid_params['speed_Kd']
                    self.ackermann_controller_settings.accel_kp = pid_params['accel_Kp']
                    self.ackermann_controller_settings.accel_ki = pid_params['accel_Ki']
                    self.ackermann_controller_settings.accel_kd = pid_params['accel_Kd']
        except KeyError:
            self.node.logwarn("PID params format error, use default values")
            return

    # TODO: ROS2 Support
    def reconfigure_pid_parameters(self, ego_vehicle_control_parameter, _level):
        """
            Callback for dynamic reconfigure call to set the PID parameters

            :param ego_vehicle_control_parameter:
            :type ego_vehicle_control_parameter: \
                carla_ackermann_control.cfg.EgoVehicleControlParameterConfig

            """

        self.node.loginfo("Reconfigure Request:  "
                         "speed ({speed_Kp}, {speed_Ki}, {speed_Kd}), "
                         "accel ({accel_Kp}, {accel_Ki}, {accel_Kd})"
                         "".format(**ego_vehicle_control_parameter))

        self.ackermann_controller_settings.speed_kp = ego_vehicle_control_parameter['speed_Kp']
        self.ackermann_controller_settings.speed_ki = ego_vehicle_control_parameter['speed_Ki']
        self.ackermann_controller_settings.speed_kd = ego_vehicle_control_parameter['speed_Kd']
        self.ackermann_controller_settings.accel_kp = ego_vehicle_control_parameter['accel_Kp']
        self.ackermann_controller_settings.accel_ki = ego_vehicle_control_parameter['accel_Ki']
        self.ackermann_controller_settings.accel_kd = ego_vehicle_control_parameter['accel_Kd']

        if self.parent.carla_actor.is_alive:
            self.parent.carla_actor.apply_ackermann_controller_settings(self.ackermann_controller_settings)


        return ego_vehicle_control_parameter

    def ackermann_cmd_callback(self, msg):
        """
        Callback for ackermann_cmd message
        """

        self.ackermann_control.steer = -msg.steering_angle

        # TODO: Handle normalization of the steering angle 
        # self.ackermann_control.steer /= 0.7        
        
        self.ackermann_control.speed = msg.speed
        self.ackermann_control.acceleration = msg.acceleration
        self.parent.carla_actor.apply_ackermann_control(self.ackermann_control)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo actor
        :return: name
        """
        return "actor.pseudo.ackermann_control"

    def on_pose(self, pose):
        if self.parent and self.parent.carla_actor.is_alive:
            self.parent.carla_actor.set_transform(ros_pose_to_carla_transform(pose))