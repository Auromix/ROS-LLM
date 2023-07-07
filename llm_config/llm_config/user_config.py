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
# This is a configuration file for a conversational AI assistant
# that uses the OpenAI API for generating responses.
#
# The user can specify the OpenAI language model to be used, the organization
# under which their API key is registered (if applicable), and several parameters
# that affect the creativity and coherence of the AI's responses, such as the
# temperature, top probability cutoff, and frequency and presence penalties.
#
# The user can also specify the prompt given to the AI, the prefix for the AI's response,
# and the maximum number of tokens and length allowed in the response.
#
# The chat history can be stored in a JSON file, with a maximum length limit.
#
# The assistant's behavior can be customized using a RobotBehavior object
# and a list of robot functions.
#
# The API credentials for Amazon AWS are provided, along with parameters
# for AWS S3, Transcribe, and Polly services, and parameters for audio recording.
#
# Author: Herman Ye @Auromix

from .robot_behavior import RobotBehavior
import os


class UserConfig:
    def __init__(self):
        # OpenAI API related
        # [required]: OpenAI API key
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        # [required]: Name of the OpenAI language model to be used
        self.openai_model = "gpt-3.5-turbo-0613"
        # self.openai_model="gpt-4-0613"
        # [optional]: Name of the organization under which the OpenAI API key is registered
        self.openai_organization = "Auromix"
        # [optional]: Controls the creativity of the AI’s responses. Higher values lead to more creative, but less coherent, responses
        self.openai_temperature = 1
        # [optional]: Probability distribution cutoff for generating responses
        self.openai_top_p = 1
        # [optional]: Number of responses to generate in batch
        self.openai_n = 1
        # [optional]: Whether to stream response results or not
        self.openai_stream = False
        # [optional]: String that if present in the AI's response, marks the end of the response
        self.openai_stop = "NULL"
        # [optional]: Maximum number of tokens allowed in the AI's respons
        self.openai_max_tokens = 4000
        # self.openai_max_tokens= 16000
        # [optional]: Value that promotes the AI to generates responses with higher diversity
        self.openai_frequency_penalty = 0
        # [optional]: Value that promotes the AI to generates responses with more information at the text prompt
        self.openai_presence_penalty = 0

        # IO related
        # [optional]: The prompt given to the AI, provided by the user
        self.user_prompt = ""
        # [optional]: The generated prompt by the administrator, used as a prefix for the AI's response
        self.system_prompt = ""
        # TODO: System prompt only works for the first message,so it will be forgotten soon after the first message
        # modify the llm_model/chatgpt.py, add system_prompt to every prompt to solve this problem @Herman Ye
        # [optional]: The generated response provided by the AI
        self.assistant_response = ""

        # Chat history related
        # [optional]: The chat history, including the user prompt, system prompt, and assistant response
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
        # [optional]: The path to the chat history JSON file
        self.chat_history_path = os.path.expanduser("~")
        # self.chat_history_path = os.path.dirname(os.path.abspath(__file__))
        # [optional]: The limit of the chat history length
        self.chat_history_max_length = 4000
        # self.chat_history_max_length=16000

        # Robot behavior related
        # [optional]: The robot behavior object
        self.robot_behavior = RobotBehavior()
        # [optional]: The robot functions list
        self.robot_functions_list = self.robot_behavior.robot_functions_list
        # [optional]: Multi-robot list
        # "" is for robot without name
        self.multi_robots_name=["turtle1","turtle2","minipupper",""]
        
        # AWS related
        # [required]: AWS IAM access key id
        self.aws_access_key_id = os.getenv("AWS_ACCESS_KEY_ID")
        # [required]: AWS IAM secret access key
        self.aws_secret_access_key = os.getenv("AWS_SECRET_ACCESS_KEY")
        # [required]: AWS IAM region name
        self.aws_region_name = 'ap-southeast-1'
        # [required]: AWS S3 bucket name
        self.bucket_name = 'auromixbucket'
        # [optional]: AWS transcription language, change this to 'zh-CN' for Chinese
        self.aws_transcription_language = "en-US"
        # [optional]: AWS polly voice id, change this to 'Zhiyu' for Chinese
        self.aws_voice_id = "Ivy"

        # OpenAI Whisper Model size related
        # [optional]: OpenAI Whisper Model size: tiny base small medium large
        self.whisper_model_size = "medium"
        # [optional]: OpenAI Whisper Model language: en
        self.whisper_language="en"
        # Audio recording related
        # [optional]: Audio recording duration, in seconds
        self.duration = 5
        # [optional]: Audio recording sample rate, in Hz
        self.sample_rate = 16000
        # [optional]: Audio recording gain multiplier
        # Change this to increase or decrease the volume
        self.volume_gain_multiplier = 1
