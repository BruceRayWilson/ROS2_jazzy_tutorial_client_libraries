docker run -it \
    --name ros2_jazzy_tutorial_client_libraries \
    --user $(id -u):$(id -g) \
    -v $(pwd):/ros2_ws \
    ros2_jazzy_tutorial_client_libraries