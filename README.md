# ros2-cartpole-imc-pid

ROS 2 Humble implementation of an inverted cart-pole stabilization system using IMC-based PID control, featuring simulation, dynamics, and real-time balancing control.

<p align="center"> 
    <a href="https://www.ros.org/">
    <img src="images/ros2_logo.png">
    </a>
    <h3 align="center">ROS 2 Humble | Inverted Cart-Pole Stabilization using IMC-PID Control</h3> 
    <p align="center"> Real-Time Balancing & Nonlinear Control System for an Inverted Pendulum <br /> 
    <br /> 
    · 
    <a href="https://github.com/rachitsrivastava2114/ros2-cartpole-imc-pid/issues">Report Bug</a> 
    · 
    </p> 
</p>

## Table of Contents

* [Project Overview](#project-overview)
  * [Objectives](#objectives)
  * [System Architecture](#system-architecture)
* [Hardware & Software Requirements](#hardware--software-requirements)
* [ROS 2 Packages](#ros-2-packages)
* [Working Principle](#working-principle)
* [Control Strategy](#control-strategy)
* [Simulation & Visualization](#simulation--visualization)
* [Verification & Testing](#verification--testing)
* [Results](#results)
* [Future Improvements](#future-improvements)
* [Authors](#authors)
* [License](#license)

## Project Overview

This project implements an automatic stabilization system for an **Inverted Cart-Pole** using **ROS 2 Humble** and an **Internal Model Control (IMC) based PID controller**.

The system continuously monitors pendulum angle and cart position while generating corrective control actions to stabilize the pendulum around its unstable upright equilibrium point.

The project demonstrates:
- Nonlinear control systems
- Real-time robotics control
- ROS 2 communication architecture
- Closed-loop feedback stabilization
- Gazebo-based dynamic simulation

<br />
<br />

<p align = "center">
<img src="images/cart_pole.png" style="width: 45%; height: 45%;">
</p>

### Objectives

* Design and simulate an inverted cart-pole stabilization system
* Implement IMC-based PID control for balancing
* Develop ROS 2 Humble compatible control architecture
* Achieve real-time pendulum stabilization
* Visualize system behavior using Gazebo and RViz
* Analyze nonlinear control response and system stability

### System Architecture

The system is built around a ROS 2 Humble framework where the cart-pole model is simulated inside Gazebo. Sensor feedback such as pendulum angle and cart position is continuously monitored by the controller node.

An IMC-PID controller computes corrective force inputs to stabilize the pendulum in its upright position. ROS 2 publishers and subscribers manage communication between simulation, controller, and visualization nodes.

The complete architecture includes:
- Cart-pole dynamic model
- Controller node
- ROS 2 communication layer
- Gazebo simulation
- RViz visualization tools

## Hardware & Software Requirements

### Software

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo Simulator
* RViz2
* Python 3 / C++
* colcon build tools

### Optional Hardware

* Embedded controller platform
* Motor driver interface
* Encoder sensors
* Linear rail cart system

## ROS 2 Packages

* `cartpole_description`
* `cartpole_control`
* `cartpole_bringup`
* `cartpole_gazebo`
* `cartpole_msgs`

## Working Principle

* The cart-pole system starts in an unstable state
* Pendulum angle and cart position are continuously measured
* The IMC-PID controller calculates corrective force inputs
* Cart motion compensates for pendulum deviation
* The controller minimizes angular error and stabilizes the pendulum
* ROS 2 nodes exchange data in real time through topics and services
* Gazebo simulates system physics and dynamic response

<br />
<br />

<p align = "center">
<img src="images/cartpole_simulation.png">
</p>

## Control Strategy

* Internal Model Control (IMC) based PID controller is implemented
* Proportional action responds to instantaneous error
* Integral action reduces steady-state error
* Derivative action improves transient response
* IMC tuning enhances robustness and stability
* Control output is constrained within safe operating limits

## Simulation & Visualization

* Gazebo is used for physics-based cart-pole simulation
* RViz2 provides real-time visualization
* ROS 2 topics monitor:
  * Pendulum angle
  * Cart position
  * Controller output
  * System states
* rqt_graph is used for node communication analysis

## Verification & Testing

* Pendulum stabilization tested under different initial conditions
* Controller performance analyzed for varying gains
* System response evaluated for disturbance rejection
* Real-time ROS 2 communication verified
* Gazebo simulation validated for dynamic consistency

## Results

* Stable upright pendulum balancing achieved
* IMC-PID controller provides smooth stabilization
* Improved disturbance rejection observed
* Real-time ROS 2 communication functions correctly
* Gazebo simulation demonstrates realistic system behavior

## Future Improvements

* MRAC implementation
* Reinforcement learning based balancing
* Hardware implementation on embedded systems
* State estimation using Kalman filters
* Adaptive and nonlinear control methods
* FPGA-based acceleration for real-time control

## Authors

**Rachit Srivastava** <br>
Bachelor of Technology – Electronics & Communication Engineering

## License

This project is developed for academic, research, and educational purposes only.
