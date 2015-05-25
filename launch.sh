gnome-terminal -e cd ~/ros_ws &&./baxter.sh && rosrun apc recognizeObjects_server.py
gnome-terminal -e cd ~/ros_ws &&./baxter.sh && rosrun apc AnalyzeMask
gnome-terminal -e cd ~/ros_ws &&./baxter.sh && roslaunch baxter_moveit_config demo_kinect.launch 
gnome-terminal -e cd ~/ros_ws &&./baxter.sh && rosrun apc move_server
gnome-terminal -e cd ~/ros_ws &&./baxter.sh && rosrun apc heuristic_control.py
