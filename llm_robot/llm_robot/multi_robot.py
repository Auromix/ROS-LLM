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
# Author: Herman Ye @Auromix

# LLM related
import json
import time
import subprocess

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from llm_interfaces.srv import ChatGPT

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()


class MultiRobot(Node):
    def __init__(self):
        super().__init__("turtle_robot")
        # Initialize publishers dictionaries
        self.pose_publishers = {}
        self.cmd_vel_publishers = {}
        for robot_name in config.multi_robots_name:
            if robot_name == "":
                # pose publisher
                self.pose_publishers[robot_name] = self.create_publisher(
                    Pose, "/pose", 10
                )
                # cmd_vel publishers
                self.cmd_vel_publishers[robot_name] = self.create_publisher(
                    Twist, "/cmd_vel", 10
                )
            else:
                # pose publisher
                self.pose_publishers[robot_name] = self.create_publisher(
                    Pose, "/" + robot_name + "/pose", 10
                )
                # cmd_vel publishers
                self.cmd_vel_publishers[robot_name] = self.create_publisher(
                    Twist, "/" + robot_name + "/cmd_vel", 10
                )

        # Server for model function call
        self.function_call_server = self.create_service(
            ChatGPT, "/ChatGPT_function_call_service", self.function_call_callback
        )

        # Initialization publisher
        self.initialization_publisher = self.create_publisher(
            String, "/llm_initialization_state", 0
        )

        # LLM state publisher
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)

        # Initialization ready
        self.publish_string("robot", self.initialization_publisher)

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

    def call_service(self, **kwargs):
        # Warning:Only empty input service is supported
        # TODO: Add support for non-empty input service
        service_name = kwargs.get("service_name", "")
        service_type = kwargs.get("service_type", "")
        command = ["ros2", "service", "call", service_name, service_type]

        try:
            command_result = subprocess.check_output(command, stderr=subprocess.STDOUT)
            command_result = command_result.decode("utf-8")  # Convert bytes to string
        except subprocess.CalledProcessError as command_error:
            command_result = command_error.output.decode("utf-8")
        return command_result

    def publish_cmd_vel(self, **kwargs):
        """
        Publishes cmd_vel message to control the movement of all types of robots
        """
        # Get parameters
        robot_name = kwargs.get("robot_name", "")
        duration = kwargs.get("duration", 0)
        linear_x = kwargs.get("linear_x", 0.0)
        linear_y = kwargs.get("linear_y", 0.0)
        linear_z = kwargs.get("linear_z", 0.0)
        angular_x = kwargs.get("angular_x", 0.0)
        angular_y = kwargs.get("angular_y", 0.0)
        angular_z = kwargs.get("angular_z", 0.0)
        self.get_logger().debug(f"Received cmd_vel message: {kwargs}")
        # Create message
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = float(linear_y)
        twist_msg.linear.z = float(linear_z)
        twist_msg.angular.x = float(angular_x)
        twist_msg.angular.y = float(angular_y)
        twist_msg.angular.z = float(angular_z)
        self.get_logger().debug(f"Created cmd_vel message: {twist_msg}")

        # Create publisher for new robot if not exist
        if robot_name not in config.multi_robots_name:
            self.cmd_vel_publishers[robot_name] = self.create_publisher(
                Twist, f"/{robot_name}/cmd_vel", 10
            )
            self.get_logger().debug(f"Created new publisher for {robot_name}")
        # Set default robot name
        if robot_name == "":
            robot_name = "default"
            topic = "/cmd_vel"
            self.get_logger().debug(f"default robot:{robot_name}")
        else:
            topic = f"/{robot_name}/cmd_vel"

        if duration == 0:
            self.cmd_vel_publishers[robot_name].publish(twist_msg)
        else:
            # Publish message for duration
            start_time = time.time()
            while time.time() - start_time < duration:
                self.cmd_vel_publishers[robot_name].publish(twist_msg)
                time.sleep(0.1)

        # Log
        self.get_logger().info(f"Published {topic} message successfully: {twist_msg}")
        
        # Stop robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publishers[robot_name].publish(stop_msg)
        return twist_msg

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )


def main():
    rclpy.init()
    multi_robot = MultiRobot()
    try:
        rclpy.spin(multi_robot)
    except KeyboardInterrupt:
        pass
    finally:
        multi_robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
