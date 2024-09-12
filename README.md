# Shadow Dexterous Hand: Optimising Pick and Place with Depth Cameras

This repository contains the code developed in order to integrate a Shadow Hand coupled to a UR5 in the factory, for the purpose of re-feeding the production line.

📝 Note: The code was developed in Python using Robotic Operating System (ROS).

The `ZED_Docker` folder contains the necessary files and instructions to get the ZED docker container, containing the code to use the ZED camera.

The `pick_n_place` folder is a ROS package responsible for the Sahdow Hand and UR5 arm control and should be placed inside the Shadow Hand docker container, specifically at `/home/user/projects/shadow_robot/base/src`.

The `camera` folder is a ROS package responsible for the ZED camera image acquisiton and should be placed inside the ZED docker container, specifically at `/root/catkin_ws/src`.
 
Software description:
  - [`acquire_pos_rs.py`](camera/src/acquire_pos_rs.py) - utilises RealSense camera to acquire the position (x,y,z) of the factory and of object to be grabbed by Shadow Hand in `rs_camera` coordinates and publishes this position to `'obj_pos'` ROS topic;
  - [`acquire_pos_zed.py`](camera/src/acquire_pos_zed.py) - utilises ZED camera to acquire the position (x,y,z) of the factory and of object to be grabbed by Shadow Hand in `zed_camera` coordinates and publishes this position to `'obj_pos'` ROS topic;
  - [`pick_n_place_rs.py`](pick_n_place/src/pick_n_place_rs.py) - control the Shadow Hand and UR5 arm in order to perform the pick and place task based on the pick and place object positions (use with RealSense camera).
  - [`pick_n_place_zed.py`](pick_n_place/src/pick_n_place_zed.py) - control the Shadow Hand and UR5 arm in order to perform the pick and place task based on the pick and place object positions (use with ZED camera).

## How to run (RealSense)

1. Turn on the robots
   
   ⚠️ Don't forget to set Shadow's NUC ip!
   
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
    
## How to run (ZED)

Follow the RealSense tutorial replacing `pick_n_place_rs.py` by `pick_n_place_zed.py` and `acquire_pos_rs.py` by `acquire_pos_zed.py`.

    
## 📫 Contact

Developed by Bruno Santos

Feel free to reach out via email: brunosantos@fe.up.pt

Last updated in: ``12/09/2024``

