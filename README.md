# rs_unreal
RoboSherlock functionality to interact with UE4 

Test the current functionality:
````
roscore
rosbag play test.bag --loop --clock #test.bag from the RoboSherlock tutorials
rosrun robosherlock run _ae:=rs_w_bs _vis:=true
````

# rs_ue4beliefstate
RoboSherlock functionality to spawn belief state in UE4 

Test the current functionality:
````
roscore # start ros server 
rosbag play test.bag --loop --clock # play back a test.bag from the RoboSherlock tutorials
roslaunch unreal_vision_bridge unreal_vision_bridge.launch # start unreal vision bridge
roslaunch rosbridge_server rosbridge_websocket.launch # start ros bridge server
rosrun tf static_transform_publisher 0.892 -1.836 0.000   0.000 0.000 1.000 0.000 /map /ue4_world 100 # start virtual to real world transform
UE4Editor # start UE4, opens project, set up
rosrun robosherlock run _ae:=demo_ue4 _vis:=true # start robosherlock with bf-visualization capabilities  
rviz # start advanced system visualization tools 
````
