#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include<sim_gazebo/Combined.h>
#include <cmath>
#include <string>
using std::vector;
using std::string;

ros::Publisher rosPub;

void modelStatesCallback(const gazebo_msgs::ModelStates& msg){
  sim_gazebo::Combined msg_pub;
  int auv_index = -1;
  for(int i = 0; i < msg.name.size(); i++)
  {
      if (msg.name[i] == "auv") auv_index = i;
  }

  if (auv_index == -1)
  {
      ROS_ERROR_STREAM("Failed to locate auv model state.");
      return;
  }
    msg_pub.angular[0]=msg.pose[auv_index].orientation.x;
    msg_pub.angular[1]=msg.pose[auv_index].orientation.y;
    msg_pub.angular[2]=msg.pose[auv_index].orientation.z;
    msg_pub.linear[0]=msg.pose[auv_index].position.x;
    msg_pub.linear[1]=msg.pose[auv_index].position.y;
    msg_pub.linear[2]=msg.pose[auv_index].position.z;
    msg_pub.depth=msg.pose[auv_index].position.z;
    rosPub.publish(msg_pub);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_publisher");
    ros::NodeHandle nh;
    ros::Subscriber auv_orient = nh.subscribe("gazebo/model_states", 1, modelStatesCallback);
    rosPub=nh.advertise<sim_gazebo::Combined>("synchronizer/combined",1);
    double rate=30;
    ros::Rate r(rate);
    
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
