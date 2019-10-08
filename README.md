# rs_unreal
RoboSherlock functionality to interact with UE4 

Test the current functionality:
````
roscore
rosbag play test.bag --loop --clock #test.bag from the RoboSherlock tutorials
rosrun robosherlock run _ae:=rs_w_bs _vis:=true
````

# Necessary setup
  * Add a RGBDCamera actor (from the URoboVision Package)
  * Add the following tag to the RGBDCamera actor: SemLog;id,urobovision_camera; (needed as reference for ROSWorldControl)
  * Set the resolution of the RGBDCamera so it matches the resolution from your main camera (kinect 1: 640x480, 1280x960(cropped) or  kinect one: 1920x1080)
