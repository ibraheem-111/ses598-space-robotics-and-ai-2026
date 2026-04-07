# Solution: 

1. Challange 1: The 

2. Challange 2: Doing slam using RTABMAP with only RGB camera. RTABMAP is made for RGBD or Lidar based SLAM, RGB images alone are not enough to do mapping using RTABMAP. 
Solution: 
   a. I first tried in vain to make it work without depth and for that I created the rtabmap_nodepth.launch.py file. 
   b. I gave up and changed the simulation in terrain_mapping.launch.py from ` make px4_sitl gz_x500_gimbal ` to ` make px4_sitl gz_x500_depth_mono `
   c. There were errors in the rtabmap.launch.file I was not sure why, it would not start mapping. I found an example file on rtabmap github repo that used rgbd images and visual odometry from kinect to map an environment. From that I understood that it is necessary to sync odometry, depth and rgb data. This version of rtabmap is in rtabmap_new.launch.py
   d. I created ` /terrain_mapping_drone_control/px4_vehicle_odometry_to_ros_odom.py ` to combine /fmu/out/vehicle_odometry with the timestamps that I exposed from gazebo and publish that to /rtabmap/odom
   e. The transform from /odom to /base_link was still missing so I added a publishing framework for that in ` px4_vehicle_odometry_to_ros_odom `
   f. The camera frame was in ENU (East North Up) while px4 was publishing odometry in (North East Down). I learned how to do quaternion transformations using a library called transform3d to convert the frames properly. I also extracted the frame transformations from model.sdf for both the front rgbd camera ( base_link -> OakD-Lite-Modify/base_link) and the down camera (base_link -> mono_camera_link). 
   g. As a result I was able to do mapping with px4 odometry, but there was a problem I kept getting the following result: ![alt text](<Screenshot from 2026-04-04 17-27-10.png>)

      This result could have been due to two possible reasons: 
      1. px4 odometry was incorrect
      2. px4 odometry was out of sync

      To test my theory, I first turned off approx sync, and still got the same result.
      I eliminated the transformations in ` px4_vehicle_odometry_to_ros_odom.py ` node to prevent latency due to that 
      I got rid of sim_time = True and made all topics publish with global time
      I made everything follow sim time

      I still got the same result. 
      I believe that it is becuase the depth camera's point cloud calculates a different value of depth and possibly more accurate than the assumed distance from the cylinder in px4 odometry

   h. I fixed the mapping by getting rid of PX4 odometry and replacing it with VIO (Visual Inertial Odometry). I exposed the IMU topic from gazebo and mapped it to rtabmap_odom. 
   ![alt text](<Screenshot from 2026-04-04 16-38-40.png>)

   I got this result because the loop closures were not qualified, there weren't enough matching features. I increased the minimum_matching_inliers for loop closure and got an acceptable map of the cylinder in 3D. 
  
   
3. The mission.launch.py was for two cylinders and wasn't written for mapping a cylinder. 
   a. I created a fuunction that forces the drone to follow a helical path arround a cylinder while keeping its yaw facing inwards
   b. I increased the radius so that it could see more features apart from the cylinder like the rock and the rover. 

4. Aruco Tracker does not provide correct information about the location of aruco markers

a. Fixed this by adding a transform from the base_link to down facing camera. 

# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze

This ROS2 package implements an autonomous drone system for geological feature detection, mapping, and analysis using an RGBD camera and PX4 SITL simulation.

## Challenge Overview

<img width="1195" height="1020" alt="image" src="https://github.com/user-attachments/assets/6e3d9610-a63a-4949-88a1-a14166a9ed50" />

Students will develop a controller for a PX4-powered drone to efficiently search, map, and analyze 3D objects in an unknown environment. The drone must map the Perseverance rover, and land on it.

### Mission Objectives
Intermediate: 
1. Search and locate the cylinder
2. Map the cylinder in 3D
3. Land safely on top of the cylinder

Advanced (extra credit): 
Execute intermediate objective, and do the following additional tasks. 
1. Search and locate the rover
2. Map the rover in 3D
3. Land safely on top of the rover

In both cases, complete mission while logging time and energy performance. 

### Evaluation Criteria (100 points)

The assignment will be evaluated based on:
- Total time taken to complete the mission
- Total energy units consumed during operation
- Accuracy of rover 3D model
- Landing precision on rover
- Performance across 3 trials

### Key Requirements

- Autonomous takeoff and search strategy implementation
- Real-time rover detection 
- Energy-conscious path planning for mapping using SLAM 
- Safe and precise landing on the rover once mapping is complete
- Robust performance across trials

## Prerequisites

- ROS2 Humble
- PX4 SITL Simulator (Tested with PX4-Autopilot main branch 9ac03f03eb)
- RTAB-Map ROS2 package
- OpenCV
- Python 3.8+

## Repository Setup

### If you already have a fork of the course repository:

```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

### If you don't have a fork yet:

1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace

```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/terrain_mapping_drone_control .
```

### Copy PX4 Model Files

Copy the custom PX4 model files to the PX4-Autopilot folder

```bash
# Navigate to the package
cd ~/ros2_ws/src/terrain_mapping_drone_control

# Make the setup script executable
chmod +x scripts/deploy_px4_model.sh

# Run the setup script to copy model files
./scripts/deploy_px4_model.sh -p /path/to/PX4-Autopilot
```

## Building and Running

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization with your PX4-Autopilot path
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py

# OR you can change the default path in the launch file
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
```
## Extra credit -- 3D reconstruction (50 points)
Use RTAB-Map or a SLAM ecosystem of your choice to map both rocks, and export the world as a mesh file, and upload to your repo. Use git large file system (LFS) if needed. 

## License

This assignment is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0). 
For more details: https://creativecommons.org/licenses/by-nc-sa/4.0/ 
