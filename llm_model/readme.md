# llm_model
## Package Description

The `llm_model` package is a ROS package that provides a conversational interface using the OpenAI API. The package includes a node called `ChatGPTNode` which interacts with the ChatGPT service to implement conversational interactions.

## chatgpt.py

This file contains the implementation of the `ChatGPTNode` node, which is responsible for providing a conversational interface using the OpenAI API. The node implements the ChatGPT service callback function `llm_callback`, which is called whenever a client sends a request to the ChatGPT service.

The `ChatGPTNode` node also includes a client function `function_call_client` and a publisher `output_publisher`. The `function_call_client` function is used to call other functions using ROS Service, while the `output_publisher` publishes messages to a topic.

The `chatgpt.py` file also includes a function called `add_message_to_history` to update chat history records. The file writes chat history records to a JSON file using Python's JSON library.

Overall, the `chatgpt.py` file provides a ROS-based conversational interface with the OpenAI API, allowing users to interact with a chatbot.


## Usage
To test the `turtle_robot.py` file with `robot node`, use the following ROS command-line to publish a speed command that **makes the turtlesim rotate**:
```bash
ros2 service call /ChatGPT_service llm_interfaces/srv/ChatGPT '{request_text: "Let the turtlesim rotate counterclockwise at a great angular velocity of 50 rad/s and move forward at a certain linear velocity"}'
```
**Reset the turtlesim**
```bash
ros2 service call /ChatGPT_service llm_interfaces/srv/ChatGPT '{request_text: "I want the little turtle to go back to where it started."}'
```