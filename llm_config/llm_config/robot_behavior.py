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
# This file contains the behavior of the robot.
# It includes a list of functions for the robot to perform,
# such as publishing a cmd_vel message to control the movement of the robot.
# To customize the robot's behavior,
# modify the functions in this file to customize the behavior of your robot
# and don't forget to modify the corresponding real functions in llm_robot/turtle_robot.py
#
# Author: Herman Ye @Auromix

# Example robot functions list for the TurtleSim
# The user can add, remove, or modify the functions in this list
robot_functions_list_1 = [
    {
        "name": "publish_cmd_vel",
        "description": "Publish cmd_vel message to control the movement of turtlesim, including rotation and movement,only used for turtlesim,not for robotic arm",
        "parameters": {
            "type": "object",
            "properties": {
                "linear_x": {
                    "type": "number",
                    "description": "The linear velocity along the x-axis",
                },
                "linear_y": {
                    "type": "number",
                    "description": "The linear velocity along the y-axis",
                },
                "linear_z": {
                    "type": "number",
                    "description": "The linear velocity along the z-axis",
                },
                "angular_x": {
                    "type": "number",
                    "description": "The angular velocity around the x-axis",
                },
                "angular_y": {
                    "type": "number",
                    "description": "The angular velocity around the y-axis",
                },
                "angular_z": {
                    "type": "number",
                    "description": "The angular velocity around the z-axis",
                },
            },
            "required": [
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ],
        },
    },
    {
        "name": "reset_turtlesim",
        "description": "Resets the turtlesim to its initial state and clears the screen,only used for turtlesim,not for robotic arm",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "publish_target_pose",
        "description": "Publish target pose message to control the movement of arm robot, including x, y, z, roll, pitch, yaw",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "number",
                    "description": "The x position of the target pose",
                },
                "y": {
                    "type": "number",
                    "description": "The y position of the target pose",
                },
                "z": {
                    "type": "number",
                    "description": "The z position of the target pose",
                },
                "roll": {
                    "type": "number",
                    "description": "The roll of the target pose",
                },
                "pitch": {
                    "type": "number",
                    "description": "The pitch of the target pose",
                },
                "yaw": {
                    "type": "number",
                    "description": "The yaw of the target pose",
                },
            },
            "required": [
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
            ],
        },
    },
    {
        "name": "publish_target_pose",
        "description": "Publish target pose message to control the movement of arm robot, including x, y, z, roll, pitch, yaw. For example,[0.2, 0.2, 0.2, 0.2, 0.2, 0.2] is a valid target pose.",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "number",
                    "description": "The x position of the target pose",
                },
                "y": {
                    "type": "number",
                    "description": "The y position of the target pose",
                },
                "z": {
                    "type": "number",
                    "description": "The z position of the target pose",
                },
                "roll": {
                    "type": "number",
                    "description": "The roll of the target pose in radians",
                },
                "pitch": {
                    "type": "number",
                    "description": "The pitch of the target pose in radians",
                },
                "yaw": {
                    "type": "number",
                    "description": "The yaw of the target pose in radians",
                },
            },
            "required": [
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
            ],
        },
    },
]

robot_functions_list_multi_robot = [
    {
        "name": "publish_cmd_vel",
        "description": "Publish cmd_vel message to control the movement and rotation of turtlesim. This function is only compatible with turtlesim and not for robotic arm.",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_name": {
                    "type": "string",
                    "description": "Name of the robot instance that should be controlled. Valid robot names are 'turtle1','turtle2','minipupper', when no specific robot name is specified, robot_name=''",
                },
                "duration": {
                    "type": "number",
                    "description": "Duration of time (in seconds) for which the movement should be performed.",
                },
                "linear_x": {
                    "type": "number",
                    "description": "Linear velocity along the x-axis for the robot.",
                },
                "linear_y": {
                    "type": "number",
                    "description": "Linear velocity along the y-axis for the robot.",
                },
                "linear_z": {
                    "type": "number",
                    "description": "Linear velocity along the z-axis for the robot.",
                },
                "angular_x": {
                    "type": "number",
                    "description": "Angular velocity around the x-axis for the robot.",
                },
                "angular_y": {
                    "type": "number",
                    "description": "Angular velocity around the y-axis for the robot.",
                },
                "angular_z": {
                    "type": "number",
                    "description": "Angular velocity around the z-axis for the robot.",
                },
            },
            "required": [
                "robot_name",
                "duration",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ],
        },
    },
]


class RobotBehavior:
    """
    This class contains the behavior of the robot.
    It is used in llm_config/user_config.py to customize the behavior of the robot.
    """

    def __init__(self):
        self.robot_functions_list = robot_functions_list_multi_robot


if __name__ == "__main__":
    pass
