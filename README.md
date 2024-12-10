## to attach camera
1. take out small corner part out of the realsense camera so you can use the usb c port
2. screw out the top portion of tripod and screw in realsense camera in
3. use wire to connect usb c of realsense to usb of pc

## to run the code: 
```
catkin_make
source devel/setup.bash
```

ssh into sawyer (alice)
```
cd ~/ros_workspaces/106a-final
catkin_make
source devel/setup.bash
```

start up camera
```
roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \
 color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \
 depth_fps:=6 color_fps:=6
```
 start up rviz and check that camera is working properly by adding new Image object under `/camera/color/image_raw`

 connect world tf frame to camera tf frame
 ```
 rosrun tf static_transform_publisher x y z 0 1.5708 0 base camera_link 100
```

 run robot python file
```
rosrun code robot_code.py
```

run motion planning
```
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
rosrun move_arm ik_example.py
```

## to run the server: (only ria's computer has access rn)
```
ssh honeydew
cd 106a-final/vision
python server.py
```

(in a separate window)
```
ngrok http 8000
```

then replace the URL in client.py with the one displayed by ngrok
