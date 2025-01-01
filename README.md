# Jazzy

## Dockerfile

The Docker file is no longer named 'Docker'.  It is now named 'Dockerfile'.

```docker
# See Docker/Dockerfile
```

## Docker Build

```bash
cd /home/wilsonb/dl/github.com/BruceRayWilson/claude-3.5-001/ros2_diff_drive_ws
git switch master
cd /home/wilsonb/dl/github.com/BruceRayWilson/claude-3.5-001/ros2_diff_drive_ws/Docker/

#docker build -t ros2_diff_drive_jazzy .
docker build -t ros2_diff_drive_jazzy --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .
```

## Docker Run

```bash
docker run -it \
    --name ros2_diff_drive_jazzy \
    -v ~/dl/github.com/BruceRayWilson/claude-3.5-001/ros2_diff_drive_ws:/ros2_diff_drive_ws \
    ros2_diff_drive_jazzy

docker run -it --user $(id -u):$(id -g) --name ros2_diff_drive_jazzy -v ~/dl/github.com/BruceRayWilson/claude-3.5-001/ros2_diff_drive_ws:/ros2_diff_drive_ws ros2_diff_drive_jazzy
```

## Docker Start

```bash
changed
docker start -i ros2_diff_drive
```

## Docker Remove Image

If necessary:

```bash
docker rmi -f ros2_diff_drive
```

## Upgrade Docker

```bash
sudo apt upgrade docker.io -y
```

## ROS2 Workspace

```bash
# Inside the container...
cd /ros2_diff_drive_ws/src

# Create a new package for the controller
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs nav_msgs tf2 tf2_ros control_msgs sensor_msgs test_msgs launch_py --description "Generic diff drive controller" --maintainer-email BruceRayWilson42@gmail.com --license MIT diff_drive_controller_generic


# Create a new package for the PWM control
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs sensor_msgs control_msgs launch_py --description "Generic PWM controller for robotics applications" --maintainer-email BruceRayWilson42@gmail.com --license MIT pwm_controller_generic

# Build the workspace
## This must be done at the workspace directory level!
cd /ros2_diff_drive_ws
colcon build

```

### Colcon Build Output

```text
Starting >>> diff_drive_controller_generic
Starting >>> pwm_controller_generic
Finished <<< diff_drive_controller_generic [0.70s]                                                      
Finished <<< pwm_controller_generic [0.70s]
                     
Summary: 2 packages finished [0.77s]
```

## Commit Changes

1. Exit container
2. git add .
3. git commit -am "After colcon build."

## Create Differential Drive Controller Node

## Create PWM Node

### Create pwm_controller_generic.py

Done.

### Next Steps

Build the package using colcon build.
Run the node using ros2 run pwm_controller_generic pwm_controller_generic.
Publish duty cycle values to the /duty_cycle topic using ros2 topic pub /duty_cycle std_msgs/msg/Float64 "data: 0.5".
Observe the PWM state on the /pwm_state topic using ros2 topic echo /pwm_state.
