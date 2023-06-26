# llm_robot

## Package Description
The `llm_robot` package provides a function interface for robots. It allows users to create a robot behavior transfer node and add the `ROS-LLM` function interface to their own robot.

## turtle_robot.py
The `turtle_robot.py` script is a ROS node that enables the control of a robot in turtlesim. The `TurtleRobot` class extends the `Node` class and defines the behavior of the turtle robot. It has the following attributes:
* A publisher for publishing `Twist` messages to control the turtle's movement.
* A client for calling an `Empty` service to reset the turtlesim.
* A server for handling requests to call functions on the turtle robot.

The `TurtleRobot` class has the following methods:
* The `function_call_callback` method is the callback function for the `ChatGPT` service. It is responsible for executing the requested function on the turtle robot and returning the result to the client.
* The `publish_cmd_vel` method publishes `Twist` messages to control the turtle's motion.
* The `reset_turtlesim` method calls the reset service using the client attribute, which resets the turtlesim to its initial state.

This example demonstrates simulating function calls for any robot,
such as controlling velocity and other service commands.
By modifying the content of this file,
A calling interface can be created for the function calls of any robot.
The Python script creates a ROS 2 Node
that controls the movement of the TurtleSim
by creating a publisher for cmd_vel messages and a client for the reset service.
It also includes a ChatGPT function call server
that can call various functions to control the TurtleSim
and return the result of the function call as a string.
