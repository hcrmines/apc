rosrun apc recognizeObjects_server.py
rosrun apc AnalyzeMask
roslaunch baxter_moveit_config demo_kinect.launch 
rosrun apc move_server
rosrun apc heuristic_control.py
