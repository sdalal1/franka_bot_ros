# BotROS 
Authors (Alphabetical): Nader Ahmed, Demiana Barsoum, Shail Dalal, Fiona Neylon, Courtney Smith

<image src="https://github.com/ME495-EmbeddedSystems/final-project-Group5/assets/144190404/a1c4235a-0d14-4e28-bbcc-6728754631ef" title="BotROS portrait"/>

<video src="https://github.com/ME495-EmbeddedSystems/final-project-Group5/assets/144190404/b2075725-eae9-4ce6-a66d-7d366f441078" title="BotROS at work" >
</video>

## Overview
The aim of this project was to paint dot an image using the Emika Franka Panda 7 DOF robotic arm. The system utilizes computer vision and color detection to identify the location of the brushes and various paint colors on a palette. The robot then plans and excecutes trajectories using a custom MoveIt API. 

## How To Run
1. Connect to the Franka and realsense camera
2. Access the robot arm through the terminal: ssh into `ssh student@station`.
3. On `station`, run `run ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot` to start the controllers and moveit.
4. On laptop, `cd` into workspace containing our packages.

% can we have nader write about to how to load an image and create the csv file for it?
% need to mention the layout of our workspace by looking at an image? or do we not need to go that much in detail?

5. In `mattagascar\mattagascar\iliketomoveitmoveit.py`, edit `file_name` to match and load the saved path of the `.csv` file of desired image. 
6. On laptop from the workspace containing our packages, run `ros2 launch listen_apriltags aprilTags.launch.xml` to start the realsense camera, rviz, and the april_tag node.
    - Before continuing, make sure there are no warnings within the tf tree in rviz and proceed
7. Also in the workspace containing the packages, run `ros2 run CvBridge depth` to start the color detection of the paint locations.  
8. Lastly, from the folder containing the desired image csv, run `ros2 run mattagascar iliketomoveitmoveit`
9. Finally, sit back and enjoy the robot painting :)

## Overall System Architecture and High Level 
### Packages:

`frankastein` is a custom Python API for MoveIt. It is used for path planning and executing the planned trajectory. 

`mattagascar` package utilizes the frankastein API to interface with the franka and other packages via the `iliketomoveitmoveit` node.

`listen_apriltags` package moderates broadcasts the april tags into the tf tree. This package also publishes the location of the paint brushes.

`listen_apriltags_interfaces` consists of the custom msg to interact with the brush location found via april tag

*** `take_picture` package creates the location of the paint dots from a specific image using a Canny edge detector and OpenCV to descritize the outline.

*** `take_picture_interfaces` contains the custom message type to publish the dot locations.

`CvBridge` package identifies the location of each paint color. This node listens to the tf tree to find the palette location. After recieving palette location, it uses color detection within a specified radius to narrow the field of detection for each color.

### Nodes

`iliketomoveitmoveit` - 

`listener` - 

`picture_node` - 

`ImageListener` -

`Depth` - 

### Custom Messages

`listen_apriltags_interfaces/msg/Loc` - 

`take_picture_interfaces/msg/Waypoint` -

`take_picture_interfaces/msg/WaypointList` -


## Algorithms Used/Lessons Learned/Future Works





