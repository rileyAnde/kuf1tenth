# KUF1Tenth Racing Repository

## Setup
1. Properly install ROS2 and all its respective dependencies.
2. Create a ROS2 workspace.
3. Clone [F1Tenth Gym](https://github.com/f1tenth/f1tenth_gym_ros) ROS into `src`
4. Clone KUF1Tenth into `src`
5. Use `git checkout <branch_name>` to access different banches of this repository.
6. From the root of the workspace, run `colcon build`
7. Source underlay and overlay for ROS2 (`source install/local_setup.bash; source /opt/ros/<ROS VERSION>/setup.bash` Replace ROS VERSION with your ROS2 version.)
## Running algorithms
1. Run the sim using [these instructions](https://github.com/f1tenth/f1tenth_gym_ros)
2. Use `ros2 run kuf1tenth <algorithm_name>`
### Available Algorithms
1. `ftg` - Follow the gap algorithm adapted from [F1Tenth Benchmarks Implementation](https://github.com/BDEvan5/f1tenth_benchmarks)
2. `tln` - [TinyLidarNet](https://github.com/CSL-KU/TinyLidarNet), a deep learning model based on NVIDIA's DAVE-2 Architecture
