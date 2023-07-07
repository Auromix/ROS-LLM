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
#
# Node test Method:
# ros2 run llm_input llm_audio_input_local
# ros2 topic echo /llm_input_audio_to_text
# ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
#
# Author: Herman Ye @Auromix


# Open Whisper related
import whisper

# Audio recording related
import sounddevice as sd
from scipy.io.wavfile import write

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()


class AudioInput(Node):
    def __init__(self):
        super().__init__("llm_audio_input")
        # tmp audio file
        self.tmp_audio_file = "/tmp/user_audio_input.flac"

        # Initialization publisher
        self.initialization_publisher = self.create_publisher(
            String, "/llm_initialization_state", 0
        )

        # LLM state publisher
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)

        # LLM state listener
        self.llm_state_subscriber = self.create_subscription(
            String, "/llm_state", self.state_listener_callback, 0
        )

        self.audio_to_text_publisher = self.create_publisher(
            String, "/llm_input_audio_to_text", 0
        )
        # Initialization ready
        self.publish_string("llm_audio_input", self.initialization_publisher)

    def state_listener_callback(self, msg):
        if msg.data == "listening":
            self.get_logger().info(f"STATE: {msg.data}")
            self.action_function_listening()

    def action_function_listening(self):
        # Recording settings
        duration = config.duration  # Audio recording duration, in seconds
        sample_rate = config.sample_rate  # Sample rate
        volume_gain_multiplier = config.volume_gain_multiplier  # Volume gain multiplier

        # Step 1: Record audio
        self.get_logger().info("Start local recording...")
        audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1
        )
        sd.wait()  # Wait until recording is finished

        # Step 2: Increase the volume by a multiplier
        audio_data *= volume_gain_multiplier

        # Step 3: Save audio to file
        write(self.tmp_audio_file, sample_rate, audio_data)
        self.get_logger().info("Stop local recording!")

        # action_function_input_processing
        self.publish_string("input_processing", self.llm_state_publisher)

        # Step 4: Process audio with OpenAI Whisper
        whisper_model = whisper.load_model(config.whisper_model_size)

        # Step 6: Wait until the conversion is complete
        self.get_logger().info("Local Converting...")

        # Step 7: Get the transcribed text
        whisper_result = whisper_model.transcribe(self.tmp_audio_file,language=config.whisper_language)

        transcript_text = whisper_result["text"]
        self.get_logger().info("Audio to text conversion complete!")

        # Step 8: Publish the transcribed text to ROS2
        if transcript_text == "":  # Empty input
            self.get_logger().info("Empty input!")
            self.publish_string("listening", self.llm_state_publisher)
        else:
            self.publish_string(transcript_text, self.audio_to_text_publisher)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)

    audio_input = AudioInput()

    rclpy.spin(audio_input)

    audio_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
