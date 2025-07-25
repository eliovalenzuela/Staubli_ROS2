Primera prueba: lanzar RVIZ con joints simulados:
	- ros2 run joint_state_publisher_gui joint_state_publisher_gui
	- ros2 launch staubli_tx2_90_description display.launch.py


FALLO RVIZ Y WSL:

- lanzar VcXsrv en windows
$ echo "export DISPLAY=:0" >> ~/.bashrc
$ source ~/.bashrc
$ echo 'export LIBGL_ALWAYS_INDIRECT=0' >> ~/.bashrc
 

ACTUALIZAR REPO:
cd src/Staubli_ROS2/
git pull origin Staubli_tx290_ROS2
rm -rf build install log
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash