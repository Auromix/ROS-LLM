## llm_config
## Package Description
The `llm_config` package configures the behavior and user preferences of a robot. It consists of two functionalities, the `robot_behavior.py` and `user_config.py` modules.

## robot_behavior.py
The `robot_behavior.py` module controls the movement of the robot by customizing a list of robot functions. To modify the robot's behavior, the functions in this module must be edited correspondingly with the `llm_robot/turtle_robot.py` real functions.

## user_config.py
The `user_config.py` module allows users to specify various parameters that affect the AI's output, including the OpenAI language model used, temperature, top probability cutoff, and frequency and presence penalties. Users can also set the chat prompt, the prefix for the AI's response, and the maximum number of tokens and length allowed in the response. The module can store chat history in a JSON file with a maximum length limit. it also enables further customization of the assistant's behavior using a RobotBehavior object and a list of robot functions. This module contains API credentials for Amazon AWS along with parameters for services such as AWS S3, Transcribe, and Polly, as well as audio recording.

For instructions on usage and customization details, please refer to the individual module files.