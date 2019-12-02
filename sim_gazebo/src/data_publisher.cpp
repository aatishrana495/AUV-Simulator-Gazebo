#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include<sim_gazebo/Combined.h>
#include <cmath>
#include <string>
using std::vector;
using std::string;

class DataPublisher
{
public:
  DataPublisher(){
    auvState_listener=nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &DataPublisher::modelStatesCallback,this);
    auvState_publisher=nh.advertise<sim_gazebo::Combined>("synchronizer/combined",100);
  }
  void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber auvState_listener;
  ros::Publisher auvState_publisher;
  int auv_index = -1;
  std::vector<float> msg_sub;

};

void DataPublisher::modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg){

  msg_sub.clear();

  for(int i = 0; i < msg->name.size(); i++)
  {
      if (msg->name[i] == "auv") auv_index = i;
  }

  if (auv_index == -1)
  {
      ROS_ERROR_STREAM("Failed to locate auv model state.");
      return;
  }

  msg_sub.push_back(msg->pose[auv_index].position.x);
  msg_sub.push_back(msg->pose[auv_index].position.y);
  msg_sub.push_back(msg->pose[auv_index].position.z);
  msg_sub.push_back(msg->pose[auv_index].orientation.x);
  msg_sub.push_back(msg->pose[auv_index].orientation.y);
  msg_sub.push_back(msg->pose[auv_index].orientation.z);
  sim_gazebo::Combined new_msg;
  new_msg.linear = {msg_sub.at(0),msg_sub.at(1),msg_sub.at(2)};
  new_msg.angular={msg_sub.at(3),msg_sub.at(4),msg_sub.at(5)};
  new_msg.depth=msg_sub.at(2);
  auvState_publisher.publish(new_msg);
  std::cout<<new_msg<<"\n";
}

int main(int argc, char** argv){
  ros::init(argc, argv, "data_publisher");
  DataPublisher dataPublisher;
  ros::Rate r(10);
  while(ros::ok()){
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
