用RVIZ查看PrRobot
roslaunch pr_description view_prrobot.launch
（打开的是prrobot.urdf.xacro）


可用Params
lidar_enabled = true or false
zed_enabled = true or false

example:
roslaunch pr_description view_prrobot.launch lidar_enabled:=true zed_enabled:=true


