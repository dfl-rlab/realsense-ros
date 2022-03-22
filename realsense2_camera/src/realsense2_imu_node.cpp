#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class RealsenseImu
{
private:
  sensor_msgs::Imu last_accel_msg;
  ros::Publisher imu_pub_;
  ros::Subscriber accel_sub_;
  ros::Subscriber gyro_sub_;
  std::string accel_topic_ = "/camera/accel/sample";
  std::string gyro_topic_ = "/camera/gyro/sample";
  std::string imu_topic_ = "/imu";

public:
  RealsenseImu()
  {
    ros::NodeHandle local_nh("~");
    local_nh.getParam("accel_topic", accel_topic_);
    local_nh.getParam("gyro_topic", gyro_topic_);
    local_nh.getParam("imu_topic", imu_topic_);

    /// Subscribers
    accel_sub_ = local_nh.subscribe(accel_topic_ ,10, &RealsenseImu::accelCallback, this);
    gyro_sub_ = local_nh.subscribe(gyro_topic_,10, &RealsenseImu::gyroCallback, this);

    /// Publishers
    imu_pub_ = local_nh.advertise<sensor_msgs::Imu>(imu_topic_,1);
  }
protected:
  void accelCallback(const sensor_msgs::Imu& accel_msg)
  {
    /// ROS_INFO_STREAM("Accel msg received ");
   last_accel_msg = accel_msg;
  }
  void gyroCallback(const sensor_msgs::Imu& gyro_msg)
  {
    /// ROS_INFO_STREAM("Gyro msg received");
    sensor_msgs::Imu imu_msg;
    imu_msg.header= gyro_msg.header;
    imu_msg.angular_velocity = gyro_msg.angular_velocity;
    imu_msg.angular_velocity_covariance = gyro_msg.angular_velocity_covariance;
    imu_msg.linear_acceleration = last_accel_msg.linear_acceleration;
    imu_msg.linear_acceleration_covariance = last_accel_msg.linear_acceleration_covariance;
    imu_pub_.publish(imu_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense2_imu_node");
  RealsenseImu imu_node;
  ros::spin();
  return 0;
}
