Visualizzare controlli attivi:
ros2 control list_controllers

Visualizzare stato dei giunti:
ros2 topic echo /joint_states

Visualizzare stringa dei giunti in arm_controller:
ros2 param get /head_controller joints

Aprire la GUI per Rviz:
ros2 run joint_state_publisher_gui joint_state_publisher_gui

Aprire la telecamera del TIAGo:
ros2 run rqt_image_view rqt_image_view → settare il topic corretto dalla GUI della camera (head_front_camera/rgb/image_raw)

Spostare la testa del TIAGo da terminale:
ros2 topic pub /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
joint_names: [‘head_1_joint’, ‘head_2_joint'], points: [{positions: [-2.17e-05, -0.57], time_from_start: {sec:
2}}]} "

Build dell’ambiente:
colcon build

Build di uno specifico pacchetto:
colcon build --packages-select <my_py_pkg>

Prima di aprire gazebo:
source ~/tiago_public_ws/install/setup.bash

Aprire il TIAGo:
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True

Launch del publisher:
ros2 run tiago_control tiago_head_scan_publisher

Launch del arcuo_detection:
ros2 run tiago_control aruco_detection

Run del launch file:
cd tiago_public_ws
colcon build --packages-select my_bringup
source ~/.bashrc
ros2 launch my_bringup action_app.launch.py


