# AUV-Simulator

![Screenshot from 2019-09-15 11-06-24](https://user-images.githubusercontent.com/39316548/64917048-33cc9080-d7a9-11e9-8495-03ade9b708e2.png)


A simulator used to test control algorithms written for an Autonomous Underwater Vehicle

This simulator was developed as a part of my work in Team Tiburon, the autonomous underwater vehicle team of NIT Rourkela. It is developed using Gazebo7.

Move the contents of model_files.zip into ~/.gazebo/models/

STEPS:

- roscore
- roslaunch sim_gazebo sauvc.roslaunch
- rosrun sim_gazebo data_publisher
- rosrun simgazebo_image_recieve simgazebo_image_recieve
