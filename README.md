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
roscore
rosbag play test.bag --loop --clock #test.bag from the RoboSherlock tutorials
rosrun robosherlock run _ae:=demo_ue4 _vis:=true
````
