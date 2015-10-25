# kuka_interface_packages
This repo includes packages needed to control the KUKA Robots with a modular interface. (i.e. as a seperate "KUKA" bridge mode) in ROS with robot-toolkit.

###Dependencies
- [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit) (from epfl-lasa)
- [motion-generators](https://github.com/epfl-lasa/motion-generators) (from epfl-lasa)

---
###Funcionality

In this package a bridge control interface for the KUKA robot using robot-toolkit. This way, instead of having your controllers implemented within the lwr-interface framework (as most robot-toolkit examples) we have a modular architecture, where control policies and motion planner can be implemented in a seperate node and the ```kuka_fri_bridge``` deals with setting the control modes and bi-directional communication with the robot. A flow chart of this can be seen below: 

---
###Usage
