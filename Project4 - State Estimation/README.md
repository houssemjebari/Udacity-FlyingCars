# Estimation Project #

In this project we will develop a cascaded PID controller for the Quadcopter and test it in different scenarios. The control architecture is shown in the following figure
<p align="center">
<img src="images/Kalman Filter Architecture.png"/>
</p>



### The Code ###

For the project, `src/QuadEstimatorEKF.cpp` file contains all of the code for the State Estimator.

All the configuration files for the controller and the vehicle are in the `config` directory.  For example, for all your control gains and other desired tuning parameters, there is a config file called `QuadControlParams.txt` set up for you.  An import note is that while the simulator is running, you can edit this file in real time and see the affects your changes have on the quad!


## The Simulator ##

In the simulator window itself, you can right click the window to select between a set of different scenarios that are designed to test the different parts of your controller.

The simulation (including visualization) is implemented in a single thread.  This is so that you can safely breakpoint code at any point and debug, without affecting any part of the simulation.

Due to deterministic timing and careful control over how the pseudo-random number generators are initialized and used, the simulation should be exactly repeatable. This means that any simulation with the same configuration should be exactly identical when run repeatedly or on different machines.

Vehicles are created and graphs are reset whenever a scenario is loaded. When a scenario is reset (due to an end condition such as time or user pressing the ‘R’ key), the config files are all re-read and state of the simulation/vehicles/graphs is reset -- however the number/name of vehicles and displayed graphs are left untouched.

When the simulation is running, you can use the arrow keys on your keyboard to impact forces on your drone to see how your controller reacts to outside forces being applied.

## Keyboard / Mouse Controls ##

There are a handful of keyboard / mouse commands to help with the simulator itself, including applying external forces on your drone to see how your controllers reacts!

 - Left drag - rotate
 - X + left drag - pan
 - Z + left drag - zoom
 - arrow keys - apply external force
 - C - clear all graphs
 - R - reset simulation
 - Space - pause simulation

## Results ##
In this part, We commanded the drone for a rectangular trajectory in space to assess the correctness of the Kalman Filter Estimator and the architecture used.
<p align="center">
<img src="images/predict-slow-drift.png" width="500"/>
</p>
