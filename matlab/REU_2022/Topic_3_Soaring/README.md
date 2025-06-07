# UAV Thermal Soaring MATLAB Simulation

This project was completed as a part of the robotics REU (Research Experience for Undergraduates) held over the summer of 2022 at West Virginia University. All code was written by Adam Pooley, Max Gao, Arushi Sharma, and Sachi Barnaby, with support from other REU participants, WVU robotics faculty, and the graduate students under the leadership of Yu Gu.

The results of this simulation software were published in:  
[**Analysis of UAV Thermal Soaring via Hawk-Inspired Swarm
Interaction**](https://www.mdpi.com/2199878)

This research was supported in part by the National Science Foundation grant number 1851815.

## High-Level Project Description

### Motivation

This simulation software was developed to analyze the behavior of swarms of autonomous UAVs (unmanned aerial vehicles), operating in thermal updrafts. Thermal updrafts are naturally forming pockets of ascending warm air, used by birds, gliders, and UAVs to ascend without powered thrust. As discussed in the previously linked publication, UAV swarms have many applications, but little research has explored a UAV swarm's exploitation of thermal updrafts.

### Simulation Content

Within the scope of a single simulation, all UAVs (also refered to as "agents") will be randomly spread around a map containing thermal updrafts. All agents individually assess their environment and use a controller to determine their next action. Agents "die" when they collide with the ground or with another agent. Within the scope of this project, the goal of the agent controllers were to keep the agents "alive" for as long as possible, making use of thermal updrafts.

### Implementation

This project was developed in MATLAB R2021b and allows simulations to be defined and executed in large batches. Each simulation is completely defined with ~80 parameters, influencing all agent behavior, thermal updraft generation, and RNG (random number generation). A fun (and desireable) feature from this architecture is that two simulations executed with the same parameters will produce the *exact* same results.

## Repository Organization

This repository is divided into the following folders:
- [Analysis](Analysis/README.md): Contains code used to analyze simulation data for trends.
- [Megaruns](Megaruns/README.md): Contains simulation data from extremely large batches ("megaruns").
- [Misc](Misc/README.md): Contains code used to clean batch simulation data.
- [Simulation](Simulation/README.md): Contains all code used to generate and vary simulations. README contains instructions to run and edit the simulation.

## Contact

Adam Pooley can be contacted for further information at pooleya19@gmail.com.