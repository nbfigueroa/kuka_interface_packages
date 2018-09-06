# kuka_interface_packages
This repo includes packages needed to control the KUKA Robots with a modular interface. (i.e. as a seperate "KUKA" bridge mode) in ROS with robot-toolkit.

###Dependencies
- [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit) (from epfl-lasa)
- [motion-generators](https://github.com/epfl-lasa/motion-generators) (from epfl-lasa)

---
### Funcionality

This package offers a bridge control interface for the KUKA robot using robot-toolkit. This way, instead of having your controllers implemented within the lwr-interface framework (as most robot-toolkit examples do), we facilitate a modular architecture, where control policies/motion planners/joint contollers can be implemented in a seperate node and the ```kuka_fri_bridge``` deals with setting the control modes and bi-directional communication with the robot. It provides two types of control modes for your convenience.

#### 1. Joint Impedance Control Mode:

The joint impedance control mode has two control interfaces: ***position*** and ***velocity***. Following is a diagram of the control flow. 

![alt tag](https://cloud.githubusercontent.com/assets/761512/10713622/224bc630-7ac1-11e5-96cd-ef2b83aa87cb.png)

From your own node you have to publish **joint commands** which can be position+stiffness or velocity+stiffness. The bridge will then publish the current **joint state** to the standard ```sensor_msgs/JointState``` message with topic name ```/joint_states``` which includes positions/velocities/effort. 

If you want to read the joint stiffness as well, you can subscribed to the topic named ```/joint_imp_states```, which is of our custom type ```kuka_fri_bridge/JointStateImpedance``` and includes all the measurements from former message + joint stiffnesses.

Similarly, there are two message options for the joint commands:  
  1. A ```sensor_msgs/JointState``` message type which has to be published  to ```/KUKA/joint_cmd```. With this message you can only set position/velcoties. The stiffness will be set with the default values (500 Nm/rad).
  2. A ```kuka_fri_bridge/JointStateImpedance``` message type which has to be published to ```/KUKA/joint_imp_cmd```. With this message you **have to** set position/velocity and stiffness. 
  
#### Set-up:
To configure the bridge for either position or velocity control interface, you have to modify one line in the ```/kuka_fri_bridge/config/KUKABridge.xml```:
```
<Options>
            <CtrlInterface> velocity </CtrlInterface>
</Options>
```

#### Running the bridge:
```
rosrun kuka_fri_bridge run_lwr.sh
```

Once this is done, run the script in the teach pad correctly until you get the message ```FRI Succesfully opened```, then you must press the ```PageDown`` key twice and afterwords type:
```
KUKA> control 1
```
in the ```kuka_fri_bridge``` terminal.  The bridge will now be waiting for the script to run on the teachpad. Keep clicking on the green button until ```JOINTIMPEDANCE``` control mode is set.

####2. Cartesian Impedance Control Mode: 

This control mode has only 1 interface posibility and it is by sending pose (position/orientation), forces/torques and stiffness values for the end-effector, these are expected to be published in the following topics:
  ```
  /KUKA/des_ee_pose
  /KUKA/des_ee_ft
  /KUKA/des_ee_stiff
  ```
![alt tag](https://cloud.githubusercontent.com/assets/761512/10713605/6167fbfa-7ac0-11e5-95c9-523ffbbf7db5.png)

As seen in the diagram, besides the joint state message described above, the bridge will also publish cartesian state messages:
  ```
  /KUKA/Pose
  /KUKA/FT
  /KUKA/STiff
  ```
The names are self-explanatory.

#### Running the bridge:
```
rosrun kuka_fri_bridge run_lwr.sh
```
Run the script in the teach pad correctly until you get the message ```FRI Succesfully opened```, the you must type the ```PageDown`` key twice and afterwords type:
```
KUKA> control 2
```
in the ```kuka_fri_bridge``` terminal.  The bridge will now be waiting for the script to run on the teachpad. Keep clicking on the green button until ```CARTESIANIMPEDANCE``` control mode is set.

***NOTE:*** If you decide to use this control mode, the robot might do some unexpected behaviors since the IK/ID is being computed in the KUKA control box. One way to be safe is to check your next cartesian command with an IK/ID solver OR if you wish to trust the robot blindly it's on you, just try not to break it or any walls.
