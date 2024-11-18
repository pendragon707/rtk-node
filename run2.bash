source install/setup.sh
source /opt/ros/humble/setup.bash
ros2 run image_transport republish  compressed raw --ros-args -r in/compressed:=/image_raw/compressed -r out:=/image_raw/uncomp
