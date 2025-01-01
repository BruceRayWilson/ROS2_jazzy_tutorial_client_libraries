docker run -it \
    --name ros2_diff_drive_jazzy \
    --user $(id -u):$(id -g) \
    -v $(pwd):/ros2_diff_drive_ws \
    ros2_diff_drive_jazzy