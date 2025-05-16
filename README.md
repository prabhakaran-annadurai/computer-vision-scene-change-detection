# Scene-Change-Detection-Model

**Robotic Vision Scene Understanding Challenge**

As part of this project, I attempted solving scene change detection task from **“The Robotic vision scene understanding challenge”** hosted by Australian Center for Robotic Vision. 
A simulation environment was constructed using Gazebo and a model robot was created using Robot Operating System 2. 
A pretrained 3D convolutional neural network based solution is proposed to detect the changes. 
The robot was moved in different types of paths and generated video of the environment. 
These video files are used to train the model to understand the scene and detect any changes.

**My learnings as part of this project**

1. ROS2 (Robot Operating System)
2. Gazebo (Open source 2D/3D simulator)
3. Simple robot creation, RGB camera plugin integration
4. Moving robot in a simulation environment & record scenes using opencv
5. Training CNN models for video files

**Future work**

1. Non-Deterministic path (current path is deterministic with only small random deviations)
2. Using LiDAR (Laser beams to measure distance and movement)
3. Tyring different algorithms and not just relying on CNN
