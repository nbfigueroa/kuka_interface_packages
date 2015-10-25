# kuka_interface_packages
This repo includes packages needed to control the KUKA Robots with a modular interface. (i.e. as a seperate "KUKA" bridge mode) in ROS with robot-toolkit.

###Dependencies
- [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit) (from epfl-lasa)
- [motion-generators](https://github.com/epfl-lasa/motion-generators) (from epfl-lasa)

---
###Funcionality

This package offers a bridge control interface for the KUKA robot using robot-toolkit. This way, instead of having your controllers implemented within the lwr-interface framework (as most robot-toolkit examples do), we facilitate a modular architecture, where control policies/motion planners/joint contollers can be implemented in a seperate node and the ```kuka_fri_bridge``` deals with setting the control modes and bi-directional communication with the robot. 

It has two types of control modes.

  **1. Joint Impedance Control Mode:** The joint impedance control mode has two control interfaces position and velocity. This has to be set in ```/kuka_fri_bridge/config/KUKABridge.xml``` in the following lines of text:
  ```
  <Options>
            <CtrlInterface> velocity </CtrlInterface>
  </Options>
  ```
  Following is diagram of the control flow. From your own node you have to publish ***joint commands*** which can be position+stiffness or velocity+stiffness. The bridge will then publish the current ***joint state*** to the standard ```sensor_msgs/JointState``` message with topic name: ```/joint_states```.
![alt tag](https://cloud.githubusercontent.com/assets/761512/10713622/224bc630-7ac1-11e5-96cd-ef2b83aa87cb.png)
    If you only want to set either position/velocity but not stiffness you can use the standard   ```sensor_msgs/JointState``` message and publish to ```/KUKA/joint_cmd```.
    If you want to modulate the stiffness then you should use our custom messages ```kuka_fri_bridge/JointStateImpedance``` and publish to ```/KUKA/joint_imp_cmd```.  
  

  
  **2. Cartesian Impedance Control Mode:** This control mode has only 1 interface posibility and it is by sending pose (position/orientation), forces/torques and stiffness values for the end-effector, these are expected to be published in the following topics:
  ```
  /KUKA/des_ee_pose
  /KUKA/des_ee_ft
  /KUKA/des_ee_stiff
  ```
![alt tag](https://cloud.githubusercontent.com/assets/761512/10713605/6167fbfa-7ac0-11e5-95c9-523ffbbf7db5.png)

