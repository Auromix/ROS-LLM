#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This example demonstrates simulating function calls for any robot,
# such as controlling velocity and other service commands.
# By modifying the content of this file,
# A calling interface can be created for the function calls of any robot.
# The Python script creates a ROS 2 Node
# that controls the movement of the TurtleSim
# by creating a publisher for cmd_vel messages and a client for the reset service.
# It also includes a ChatGPT function call server
# that can call various functions to control the TurtleSim
# and return the result of the function call as a string.
#
# Author: Herman Ye @Auromix

# ROS related
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import ChatGPT
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import Empty

# LLM related
import json
import os
from llm_config.user_config import UserConfig

# Global Initialization
config = UserConfig()


class ArmRobot(Node):
    def __init__(self):
        super().__init__("arm_robot")

        # Publisher for target_pose
        self.target_pose_publisher = self.create_publisher(
            Float64MultiArray, "/target_pose", 10
        )

        # Server for function call
        self.function_call_server = self.create_service(
            ChatGPT, "/ChatGPT_function_call_service", self.function_call_callback
        )
        # Node initialization log
        self.get_logger().info("ArmRobot node has been initialized")

    def function_call_callback(self, request, response):
        req = json.loads(request.request_text)
        function_name = req["name"]
        function_args = json.loads(req["arguments"])
        func_obj = getattr(self, function_name)
        try:
            function_execution_result = func_obj(**function_args)
        except Exception as error:
            self.get_logger().info(f"Failed to call function: {error}")
            response.response_text = str(error)
        else:
            response.response_text = str(function_execution_result)
        return response

    def publish_target_pose(self, **kwargs):
        """
        Publishes target_pose message to control the movement of arx5_arm
        """

        x_value = kwargs.get("x", 0.2)
        y_value = kwargs.get("y", 0.2)
        z_value = kwargs.get("z", 0.2)

        roll_value = kwargs.get("roll", 0.2)
        pitch_value = kwargs.get("pitch", 0.2)
        yaw_value = kwargs.get("yaw", 0.2)

        pose = [x_value, y_value, z_value, roll_value, pitch_value, yaw_value]
        pose_str = ', '.join(map(str, pose))

        command=f"ros2 topic pub /target_pose std_msgs/msg/Float64MultiArray '{{data: [{pose_str}]}}' -1"
        os.system(command)
        self.get_logger().info(f"Published target message successfully: {pose}")
        return pose_str



def main():
    rclpy.init()
    arm_robot = ArmRobot()
    rclpy.spin(arm_robot)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
