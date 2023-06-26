[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html) &nbsp; [![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/) &nbsp; [![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/chatgpt-minipupper2-ros2-humble/blob/main/LICENSE) &nbsp;

# ROS-LLM
The ROS-LLM project is a ROS interface designed to provide natural language and visual interaction as well as motion control capabilities for your robot. This software package enables you to harness LLM-based features (such as GPT-4 and ChatGPT) to enhance robotic applications within the ROS ecosystem, including both ros1 and ros2. Through an easy-to-use installation and customization process, ROS-LLM offers a easy solution for crafting interactive experiences with any robot. 

# How to use
## 1. Clone the repository
```bash
git clone https://github.com/Auromix/ROS-LLM.git
```
## 2. Install the dependencies
```bash
cd ROS-LLM/llm_install
. dependencies_install.sh
```
## 3. Add your OpenAI API key
```bash
cd ROS-LLM/llm_install
. config_openai_api_key.sh
```
**NOTES:**
1. If you don't have an OpenAI API key, you can get one [here](https://platform.openai.com).

2. Click on the user icon in the upper-right corner.
3. Click `View API keys`.
4. Click `Create new secret key`.
5. Enter a `name` and click `Create secret key`.
6. Copy your secret key.



## 4. build the workspace
```bash
cd <your_ws>
colcon build --symlink-install
```
## 5. Run the demo
Terminal 1:
```bash
source <your_ws>/install/setup.bash
ros2 launch llm_bringup chatgpt_with_robot_test.launch.py 
```
Terminal 2:
```bash
ros2 service call /ChatGPT_service llm_interfaces/srv/ChatGPT '{request_text: "Let the turtlesim rotate counterclockwise at a great angular velocity of 50 rad/s and move forward at a certain linear velocity"}'
```
## 6. Config your own robot [optional]
Modify the llm_robot and llm_config packages to fit your robot.
You can customize behavior for your robot.

# Schedule
This project is still under development. We will release the second version in the near future. Please stay tuned.

## TODO
- [ ] Fix the `bug` of the robot to model service.

- [ ] Add the `audio input and output` function, this function has been completed and tesed, but it has not been merged into the main branch.

- [ ] Add the `model agent` function to let the model make its own decisions

## To user
If you find this project useful, please consider giving it a ⭐️ star on GitHub! Your support helps us improve the project and encourages further development. Don't forget to also share it with your friends and colleagues who might it beneficial. Thank you for your support! 

# Contributing
Contributions are welcome! Please read the [contributing guidelines](CONTRIBUTING.md) before submitting a pull request.



# License
```
Copyright 2023 HermanYe @Auromix
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
```



