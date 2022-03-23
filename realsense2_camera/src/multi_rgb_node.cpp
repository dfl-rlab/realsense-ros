#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "stereo_processor.h"
#include <cv_bridge/cv_bridge.h>

namespace contour_ros
{
class RGBNode : public StereoProcessor
{
private:
  image_transport::Publisher left_img_pub_;
  image_transport::Publisher right_img_pub_;
public:
  RGBNode(const std::string& transport)
  : StereoProcessor(transport)
  {
    ros::NodeHandle local_nh("~");
    image_transport::ImageTransport it(local_nh);
    left_img_pub_ = it.advertise("camera1/image_raw", 1);
    right_img_pub_ = it.advertise("camera2/image_raw", 1);
  }
protected:
  void imageCallback( const sensor_msgs::ImageConstPtr& l_image_msg,
                       const sensor_msgs::ImageConstPtr& r_image_msg,
                       const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                       const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
   //ROS_INFO_STREAM("Image callback");
   cv_bridge::CvImagePtr left_cv_ptr;
   left_cv_ptr = cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::RGB8);
   left_img_pub_.publish(*(left_cv_ptr->toImageMsg()));

   cv_bridge::CvImagePtr right_cv_ptr;
   right_cv_ptr = cv_bridge::toCvCopy(r_image_msg, sensor_msgs::image_encodings::RGB8);
   right_img_pub_.publish(*(right_cv_ptr->toImageMsg()));
  }

};
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "multi_rgb_node");
 if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
 {
   ros::console::notifyLoggerLevelsChanged();
 }
 else
 {
   ROS_ERROR("The logger level not changed");
 }

 if (ros::names::remap("stereo") == "stereo")
 {
   ROS_WARN("'stereo' has not been remapped!\n");
 }

 if (ros::names::remap("image").find("rect") == std::string::npos)
 {
   ROS_WARN("input image has not been remapped");
 }

 std::string transport = argc > 1 ? argv[1] : "raw";
 contour_ros::RGBNode rgb(transport);
 ros::spin();
 return 0;
}
