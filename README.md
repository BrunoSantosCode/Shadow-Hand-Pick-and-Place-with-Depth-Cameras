# 🦾 Shadow Dexterous Hand: Optimising Pick and Place with Depth Cameras

This repository contains the code developed in order to integrate a Shadow Hand coupled to a UR5 in the FESTO demonstration factory, for the purpose of re-feeding the production line.

## 📌 Project Overview
The project leverages depth cameras (RealSense and ZED) to control the Shadow Dexterous Hand and UR5 arm for pick-and-place operations. This repository provides the code necessary to set up, control, and monitor the system using **Python** and the **Robotic Operating System (ROS)**.

 - **Robotic Hand**: Shadow Dexterous Hand
 - **Robot Arm**: UR5 (Universal Robots)
 - **Cameras**: RealSense D456, ZED v1
 - **Development Environment**: ROS, Docker

## 🗂️ Folder Structure
 - **[`CAMERA_Docker`](CAMERA_Docker**: Contains files and instructions to configure the CAMERA docker container for the cameras integration.
 - **[`pick_n_place`](pick_n_plcae)**: ROS package for controlling the Shadow Hand and UR5. This package should be placed in the Shadow Hand docker container at `/home/user/projects/shadow_robot/base/src`.
 - **[`camera`](camera)**: ROS package for acquiring images from the cameras, to be used inside the CAMERA docker container, located at `/root/catkin_ws/src`.

## ⚙️ Software Description
 - [`acquire_pos_rs.py`](camera/src/acquire_pos_rs.py)  
   Uses the **RealSense** camera to acquire the position (x, y, z) of the object and the factory area. Publishes the position to the `'obj_pos'` ROS topic.
   
 - [`acquire_pos_zed.py`](camera/src/acquire_pos_zed.py)  
   Uses the **ZED** camera to acquire the position (x, y, z) of the object and factory. Publishes the position to the `'obj_pos'` ROS topic.
   
 - [`pick_n_place_rs.py`](pick_n_place/src/pick_n_place_rs.py)  
   Controls the Shadow Hand and UR5 arm to perform pick-and-place operations based on the **RealSense** camera's object position.
 
 - [`pick_n_place_zed.py`](pick_n_place/src/pick_n_place_zed.py)  
   Controls the Shadow Hand and UR5 arm to perform pick-and-place operations using the **ZED** camera's object position.


## 🚀 How to Run

### 🟢 Running with RealSense
Follow these steps to run the pick-and-place system using the **RealSense** camera:

1. Turn on the Robots
   ⚠️ Ensure that the Shadow Hand’s NUC IP is correctly set.
   
2. Execute `Launch Shadow Right Hand and Arm.desktop`

3. In `Server Docker Container` terminal run `pick_n_place_rs.py`
    ```bash
      roslaunch pick_n_place pick_n_place_rs.launch
    ```

4. Execute CAMERA container (+ info [here](/CAMERA_Docker))

5. Go to ROS workspace folder
    ```bash
      cd ~/catkin_ws
    ```

10. Build ROS workspace
    ```bash
      catkin_make
    ```

11. Source setup file
    ```bash
      source devel/setup.bash
    ```   
12. Run `acquire_pos_rs.py`
    ```bash
      roslaunch camera acquire_pos_rs.launch
    ```
    
### 🟣 Running with ZED

To run the system with the **ZED** camera, follow the same steps as for RealSense but replace the scripts:
 - Replace `pick_n_place_rs.py` by `pick_n_place_zed.py`
 - Replace `acquire_pos_rs.py` by `acquire_pos_zed.py`
    
## 📫 Contact

Developed by Bruno Santos

Feel free to reach out via email: brunosantos@fe.up.pt

Last updated in: ``12/09/2024``

