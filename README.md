# lvi--A Visual-Inertial Fusion Depth Measurement System for Hyper-Redundant Robots


## Project Overview

`lvi` is a visual-inertial fusion depth measurement system for the end-effector of hyper-redundant robots. The system combines data from visual and inertial sensors to achieve high-precision depth estimation, making it particularly suitable for applications in constrained environments.

[![OfDU1g.png](https://ooo.0x0.ooo/2025/08/13/OfDU1g.png)](https://img.tg/image/OfDU1g)
![image](https://note.youdao.com/yws/res/2/WEBRESOURCE40bfc7cf72815685f4c43e5e50bb2e92)

If you use this code, please cite our paper:

```
@ARTICLE{10619992,
  author={Song, Haoyi and Deng, Jiangqin and Guo, Weichao and Sheng, Xinjun},
  journal={IEEE Transactions on Instrumentation and Measurement}, 
  title={Visual–Inertial Fusion With Depth Measuring for Hyper-Redundant Robot’s End Under Constrained Environment}, 
  year={2024},
  volume={73},
  number={},
  pages={1-11},
  keywords={Robot sensing systems;Robots;Cameras;Robot vision systems;Measurement by laser beam;Accuracy;Optimization;Hyper-redundant robot (HRR);laser spot;scale factor;visual–inertial odometry (VIO)},
  doi={10.1109/TIM.2024.3436096}}

```


## Features

- **Visual-Inertial Fusion**: Combines data from visual and inertial sensors for high-precision depth estimation.
- **Adaptable to Constrained Environments**: Provides stable depth measurements even in constrained environments.
- **High-Precision Estimation**: Uses optimization algorithms to improve depth estimation accuracy.

## System Architecture

The system consists of the following main modules:

- **Image Processing Module**: Processes image data from the visual sensor.
- **Estimation Module**: Performs depth estimation.
- **TF Publishing Module**: Publishes the estimated results as TF information.
- **USB Camera Driver Module**: Provides support for the USB camera.
- **Xsens ROS MTi Driver Module**: Provides support for the Xsens MTi inertial sensor.

[![OfDWHB.md.png](https://ooo.0x0.ooo/2025/08/13/OfDWHB.md.png)](https://img.tg/image/OfDWHB)

## Installation Guide

1. Create a catkin workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   catkin_init_workspace
   ```
2. Clone the repository:

   ```bash
   git clone https://github.com/Haoyi-SJTU/lvi.git
   ```
3. Install dependencies.
4. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

5. Launch all the nodes:
   ```bash
   roslaunch estimator launch_estimator.launch
   ```
   In this file we launch the following nodes:
   - IMU Node
   - Camera Node
   - The ideal trace publisher from the HRR
   - Camera Processor Node
   - Estimator Node: Receive all the sensor data and process them via Gurobi Optimizer
   - Visualization Node: Visualize the estimated results in RVIZ
   
## Usage

After launching the system, you can view the depth estimation results in the following ways:

1. RViz: Subscribe to the /depth_image topic in RViz to view the depth images.

2. TF: View the /camera_depth_frame coordinate frame in RViz to observe the depth estimation results.

## Notes
Ensure sensor time synchronization for accurate depth estimation.

In constrained environments, the system may require additional tuning and optimization.

## Video Demo
[video](https://www.bilibili.com/video/BV164t9zrEYS/)

## License
This project is licensed under the MIT License. See the [LICENSE](https://github.com/Haoyi-SJTU/lvi/blob/main/LICENSE) file for more details.

## Contact
For any inquiries, feel free to contact us:

Email: songhaoyi@sjtu.edu.cn
GitHub: [Haoyi-SJTU](https://github.com/Haoyi-SJTU/)
