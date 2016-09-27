# baxter recording dataset
  ```
  sudo bash
  roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect2_1
  ```

#rosrun kinect2_viewer kinect2_viewer kinect2_1 sd cloud

#rosrun pcl_functions filter 1

#rviz

#rosrun baxter_pykdl kinect_frame.py

#rosrun joy joy_node

#rosrun baxter_tools tuck_arms.py -u
#rosrun baxter_pykdl velocty_control_joy.py 

#rosrun baxter_tools camera_control.py -o right_hand_camera -r 640x400
#rosrun baxter_tools camera_control.py -o left_hand_camera -r 640x400
#rosrun pcl_functions save_data 2


#for saving
#./xyz_cluster 3
#./pc_tracking 3 3 40 50 250 61
#./cloud_viewer 3 61

# baxter_demos
demos for baxer robot
# on server
roslaunch baxter_demos table_top_with_baxter.launch

# on my pc
rosrun baxter_demos object_relations.py

rosrun baxter_demos language_gui.py

# load rgb or data2

rosrun baxter_tools tuck_arms.py -u

rosrun baxter_demos xyz_move.py

roslaunch ros_mary_tts ros_mary.launch

rosrun baxter_voice_recognition voice_recognition.py


# gaussian models
roscore
rosrun rviz rviz
roslaunch openni_launch openni.launch load_driver:=false camera:=head_mount_kinect
rosrun baxter_demos 3d_gaussian_model_real_data.py


# object_recording
# no need to connect ot baxter just source the ros_ws/devel/setup.bash
roslaunch baxter_demos table_top_with_baxter.launch
rosrun rviz rviz
rosrun baxter_demos object_11_pc_tracking_histograms_size.py
rosrun baxter_demos language_gui_for_scene_learning.py

