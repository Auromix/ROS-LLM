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
# ros2 run llm_input llm_audio_input
# ros2 topic echo /llm_input_audio_to_text
# ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
#
# Author: Herman Ye @Auromix

# Other libraries
import datetime
import json
import requests
import time

# AWS ASR related
import boto3

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

        # AWS service initialization
        self.aws_audio_file = "/tmp/user_audio_input.flac"
        self.aws_access_key_id = config.aws_access_key_id
        self.aws_secret_access_key = config.aws_secret_access_key
        self.aws_region_name = config.aws_region_name
        self.aws_session = boto3.Session(
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.aws_region_name,
        )

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
        # AWS S3 settings
        bucket_name = config.bucket_name
        audio_file_key = "gpt_audio.flac"  # Name of the audio file in S3
        transcribe_job_name = (
            f'my-transcribe-job-{datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}'
        )
        # Name the conversion task based on time to ensure uniqueness
        # Path of the audio file in S3
        transcribe_job_uri = f"s3://{bucket_name}/{audio_file_key}"

        # Step 1: Record audio
        self.get_logger().info("Start recording...")
        audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1
        )
        sd.wait()  # Wait until recording is finished

        # Step 2: Increase the volume by a multiplier
        audio_data *= volume_gain_multiplier

        # Step 3: Save audio to file
        write(self.aws_audio_file, sample_rate, audio_data)
        self.get_logger().info("Stop recording!")

        # action_function_input_processing
        self.publish_string("input_processing", self.llm_state_publisher)
        # Step 4: Upload audio to AWS S3
        s3 = self.aws_session.client("s3")
        self.get_logger().info("Uploading audio to AWS S3...")
        with open(self.aws_audio_file, "rb") as f:
            s3.upload_fileobj(Fileobj=f, Bucket=bucket_name, Key=audio_file_key)
        self.get_logger().info("Upload complete!")

        # Step 5: Convert audio to text
        transcribe = self.aws_session.client("transcribe")
        self.get_logger().info("Converting audio to text...")
        transcribe.start_transcription_job(
            TranscriptionJobName=transcribe_job_name,
            LanguageCode=config.aws_transcription_language,
            Media={"MediaFileUri": transcribe_job_uri},
        )

        # Step 6: Wait until the conversion is complete
        while True:
            status = transcribe.get_transcription_job(
                TranscriptionJobName=transcribe_job_name
            )
            if status["TranscriptionJob"]["TranscriptionJobStatus"] in [
                "COMPLETED",
                "FAILED",
            ]:
                break

            self.get_logger().info("Converting...")
            time.sleep(0.5)

        # Step 7: Get the transcribed text
        if status["TranscriptionJob"]["TranscriptionJobStatus"] == "COMPLETED":
            transcript_file_url = status["TranscriptionJob"]["Transcript"][
                "TranscriptFileUri"
            ]
            response = requests.get(transcript_file_url)
            transcript_data = json.loads(response.text)
            transcript_text = transcript_data["results"]["transcripts"][0]["transcript"]
            self.get_logger().info("Audio to text conversion complete!")
            # Step 8: Publish the transcribed text to ROS2
            if transcript_text == "":  # Empty input
                self.get_logger().info("Empty input!")
                self.publish_string("listening", self.llm_state_publisher)
            else:
                self.publish_string(transcript_text, self.audio_to_text_publisher)
            # Step 9: Delete the temporary audio file from AWS S3
            s3.delete_object(Bucket=bucket_name, Key=audio_file_key)

        else:
            self.get_logger().error(
                f"Failed to transcribe audio: {status['TranscriptionJob']['FailureReason']}"
            )

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
