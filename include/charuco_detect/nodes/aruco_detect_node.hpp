#pragma once
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_broadcaster.h>

namespace charuco_detect
{
class ArucoPoseNode
{
public:
  /*! Create the node class with ros_param parametrization */
  ArucoPoseNode();

  /*! Callback for processing camera messages */
  void process_camera(const sensor_msgs::ImageConstPtr &image,
                     const sensor_msgs::CameraInfoConstPtr &camera_info);

private:
  ros::NodeHandle node_handle;
  // params
  double marker_length;
  std::string camera_base_topic;
  std::string camera_frame_id;
  std::string aruco_base_topic;
  // transmission helpers
  tf2_ros::TransformBroadcaster tf_broadcast;
  image_transport::ImageTransport image_transport;
  image_transport::CameraSubscriber camera_subscriber;
  image_transport::CameraPublisher aruco_publisher;
};
} // namespace charuco_detect