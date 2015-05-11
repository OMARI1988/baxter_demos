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

