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
# This code defines a ROS node called ChatGPTNode
# The node interacts with the ChatGPT service to implement conversational interactions
# The node implements the ChatGPT service callback function "llm_callback"
# The node also includes a client function "function_call_client" and a publisher "output_publisher"
# It also includes a function called "add_message_to_history" to update chat history records
# The code generates a chat response using the OpenAI API
# It extracts response information from the response data
# The code writes chat history records to a JSON file using Python's JSON library
# The code calls other functions using ROS Service
#
# Node test Method:
# ros2 run llm_model chatgpt
# ros2 topic echo /llm_feedback_to_user
# ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 'Hello,tell me a joke'" -1
#
# Author: Herman Ye @Auromix

# ROS related
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import ChatGPT
from std_msgs.msg import String

# LLM related
import json
import os
import time
import openai
from llm_config.user_config import UserConfig


# Global Initialization
config = UserConfig()
openai.api_key = config.openai_api_key
# openai.organization = config.openai_organization


class ChatGPTNode(Node):
    def __init__(self):
        super().__init__("ChatGPT_node")
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
        # LLM input listener
        self.llm_input_subscriber = self.create_subscription(
            String, "/llm_input_audio_to_text", self.llm_callback, 0
        )
        # LLM response type publisher
        self.llm_response_type_publisher = self.create_publisher(
            String, "/llm_response_type", 0
        )

        # LLM feedback for user publisher
        self.llm_feedback_publisher = self.create_publisher(
            String, "/llm_feedback_to_user", 0
        )
        # ChatGPT function call client
        # When function call is detected
        # ChatGPT client will call function call service in robot node
        self.function_call_client = self.create_client(
            ChatGPT, "/ChatGPT_function_call_service"
        )
        # self.function_call_future = None
        # Wait for function call server to be ready
        # while not self.function_call_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(
        #         "ChatGPT Function Call Server(ROBOT NODE) not available, waiting again..."
        #     )
        self.function_call_requst = ChatGPT.Request()  # Function call request
        self.get_logger().info("ChatGPT Function Call Server is ready")

        # ChatGPT output publisher
        # When feedback text to user is detected
        # ChatGPT node will publish feedback text to output node
        self.output_publisher = self.create_publisher(String, "ChatGPT_text_output", 10)

        # Chat history
        # The chat history contains user & ChatGPT interaction information
        # Chat history is stored in a JSON file in the user_config.chat_history_path
        # There is a maximum word limit for chat history
        # And the upper limit is user_config.chat_history_max_length
        # TODO: Longer interactive content should be stored in the JSON file
        # exceeding token limit, waiting to update @Herman Ye
        self.start_timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.chat_history_file = os.path.join(
            config.chat_history_path, f"chat_history_{self.start_timestamp}.json"
        )
        self.write_chat_history_to_json()
        self.get_logger().info(f"Chat history saved to {self.chat_history_file}")

        # Function name
        self.function_name = "null"
        # Initialization ready
        self.publish_string("llm_model_processing", self.initialization_publisher)

    def state_listener_callback(self, msg):
        self.get_logger().debug(f"model node get current State:{msg}")
        # TODO

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )

    def add_message_to_history(
        self, role, content="null", function_call=None, name=None
    ):
        """
        Add a new message_element_object to the chat history
        with the given role, content, and function call information.
        The message_element_object dictionary contains
        the key-value pairs for "role", "content", and "function_call".
        If the chat history exceeds the maximum allowed length,
        the oldest message_element_object will be removed.
        Returns the updated chat history list.
        """
        # Creating message dictionary with given options
        message_element_object = {
            "role": role,
            "content": content,
        }
        # Adding function call information if provided
        if name is not None:
            message_element_object["name"] = name
        # Adding function call information if provided
        if function_call is not None:
            message_element_object["function_call"] = function_call
        # Adding message_element_object to chat history
        config.chat_history.append(message_element_object)
        # Log
        self.get_logger().info(f"Chat history updated with {message_element_object}")
        # Checking if chat history is too long
        if len(config.chat_history) > config.chat_history_max_length:
            self.get_logger().info(
                f"Chat history is too long, popping the oldest message: {config.chat_history[0]}"
            )
            config.chat_history.pop(0)

        # Returning updated chat history
        return config.chat_history

    def generate_chatgpt_response(self, messages_input):
        """
        Generates a chatgpt response based on the input messages provided.
        All parameters can be found in the llm_config/user_config.py file.
        """
        # Log
        self.get_logger().info(f"Sending messages to OpenAI: {messages_input}")
        response = openai.ChatCompletion.create(
            model=config.openai_model,
            messages=messages_input,
            functions=config.robot_functions_list,
            function_call="auto",
            # temperature=config.openai_temperature,
            # top_p=config.openai_top_p,
            # n=config.openai_n,
            # stream=config.openai_stream,
            # stop=config.openai_stop,
            # max_tokens=config.openai_max_tokens,
            # presence_penalty=config.openai_presence_penalty,
            # frequency_penalty=config.openai_frequency_penalty,
        )
        # Log
        self.get_logger().info(f"OpenAI response: {response}")
        return response

    def get_response_information(self, chatgpt_response):
        """
        Returns the response information from the chatgpt response.
        The response information includes the message, text, function call, and function flag.
        function_flag = 0: no function call, 1: function call
        """
        # Getting response information
        message = chatgpt_response["choices"][0]["message"]
        content = message.get("content")
        function_call = message.get("function_call", None)

        # Initializing function flag, 0: no function call, 1: function call
        function_flag = 0

        # If the content is not None, then the response is text
        # If the content is None, then the response is function call
        if content is not None:
            function_flag = 0
            self.get_logger().info("OpenAI response type: TEXT")
        else:
            function_flag = 1
            self.get_logger().info("OpenAI response type: FUNCTION CALL")
        # Log
        self.get_logger().info(
            f"Get message from OpenAI: {message}, type: {type(message)}"
        )
        self.get_logger().info(
            f"Get content from OpenAI: {content}, type: {type(content)}"
        )
        self.get_logger().info(
            f"Get function call from OpenAI: {function_call}, type: {type(function_call)}"
        )

        return message, content, function_call, function_flag

    def write_chat_history_to_json(self):
        """
        Write the chat history to a JSON file.
        """
        try:
            # Converting chat history to JSON string
            json_data = json.dumps(config.chat_history)

            # Writing JSON to file
            with open(self.chat_history_file, "w", encoding="utf-8") as file:
                file.write(json_data)

            self.get_logger().info("Chat history has been written to JSON")
            return True

        except IOError as error:
            # Error writing chat history to JSON
            self.get_logger().error(f"Error writing chat history to JSON: {error}")
            return False

    def function_call(self, function_call_input):
        """
        Sends a function call request with the given input and waits for the response.
        When the response is received, the function call response callback is called.
        """
        # JSON object to string
        function_call_input_str = json.dumps(function_call_input)
        # Get function name
        self.function_name = function_call_input["name"]
        # Send function call request
        self.function_call_requst.request_text = function_call_input_str
        self.get_logger().info(
            f"Request for ChatGPT_function_call_service: {self.function_call_requst.request_text}"
        )
        future = self.function_call_client.call_async(self.function_call_requst)
        future.add_done_callback(self.function_call_response_callback)

    def function_call_response_callback(self, future):
        """
        The function call response callback is called when the function call response is received.
        the function_call_response_callback will call the gpt service again
        to get the text response to user
        """
        try:
            response = future.result()
            self.get_logger().info(
                f"Response from ChatGPT_function_call_service: {response}"
            )

        except Exception as e:
            self.get_logger().info(f"ChatGPT function call service failed {e}")

        response_text = "null"
        self.add_message_to_history(
            role="function",
            name=self.function_name,
            content=str(response_text),
        )
        # Generate chat completion
        second_chatgpt_response = self.generate_chatgpt_response(config.chat_history)
        # Get response information
        message, text, function_call, function_flag = self.get_response_information(
            second_chatgpt_response
        )
        self.publish_string(text, self.llm_feedback_publisher)

    def llm_callback(self, msg):
        """
        The llm_callback function is called when the ChatGPT service is called.
        llm_callback is the main function of the ChatGPT node.
        """
        # Log the llm_callback
        self.get_logger().info("STATE: model_processing")

        self.get_logger().info(f"Input message received: {msg.data}")
        # Add user message to chat history
        user_prompt = msg.data
        self.add_message_to_history("user", user_prompt)
        # Generate chat completion
        chatgpt_response = self.generate_chatgpt_response(config.chat_history)
        # Get response information
        message, text, function_call, function_flag = self.get_response_information(
            chatgpt_response
        )
        # Append response to chat history
        self.add_message_to_history(
            role="assistant", content=text, function_call=function_call
        )
        # Write chat history to JSON
        self.write_chat_history_to_json()

        # Log output_processing
        self.get_logger().info("STATE: output_processing")
        if function_flag == 1:
            # Write response text to GPT service response
            llm_response_type = "function_call"
            self.publish_string(llm_response_type, self.llm_response_type_publisher)

            # Robot function call
            # Log function execution
            self.get_logger().info("STATE: function_execution")
            self.function_call(function_call)
        else:
            # Return text response
            llm_response_type = "feedback_for_user"
            # Log feedback_for_user
            self.get_logger().info("STATE: feedback_for_user")
            self.publish_string(llm_response_type, self.llm_response_type_publisher)
            self.publish_string(text, self.llm_feedback_publisher)
            # self.publish_string(json.dumps(text), self.llm_feedback_publisher)


def main(args=None):
    rclpy.init(args=args)
    chatgpt = ChatGPTNode()
    rclpy.spin(chatgpt)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
