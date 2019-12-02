#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string FRONTCAM_WINDOW = "frontcam Image window";
static const std::string BOTTOMCAM_WINDOW = "bottomcam Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber bottomcam_sub, frontcam_sub;
  image_transport::Publisher bottomcam_pub,frontcam_pub;

public:
  ImageConverter():it_(nh_){
    // Subscrive to input video feed and publish output video feed
    frontcam_sub = it_.subscribe("/front_camera/image", 1,&ImageConverter::frontcamimageCb, this);
    frontcam_pub = it_.advertise("/front_camera/image_rect_color", 1);
    bottomcam_sub = it_.subscribe("/bottom_camera/image", 1,&ImageConverter::bottomcamimageCb, this);
    bottomcam_pub = it_.advertise("/bottom_camera/image_rect_color", 1);
    cv::namedWindow(FRONTCAM_WINDOW);
    cv::namedWindow(BOTTOMCAM_WINDOW);
  }

  ~ImageConverter(){
    cv::destroyWindow(FRONTCAM_WINDOW);
    cv::destroyWindow(BOTTOMCAM_WINDOW);
  }

void frontcamimageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Update GUI Window
  cv::imshow(FRONTCAM_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
 frontcam_pub.publish(cv_ptr->toImageMsg());
}

void bottomcamimageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Update GUI Window
  cv::imshow(BOTTOMCAM_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
 bottomcam_pub.publish(cv_ptr->toImageMsg());
}

};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
