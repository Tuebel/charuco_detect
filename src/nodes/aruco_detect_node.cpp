#include <charuco_detect/nodes/aruco_detect_node.hpp>
#include <charuco_detect/aruco/aruco_detect.hpp>
#include <charuco_detect/conversion/camera_conversion.hpp>
#include <charuco_detect/conversion/pose_conversion.hpp>
#include <charuco_detect/visualization/draw_pose.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace charuco_detect
{
ArucoPoseNode::ArucoPoseNode() : image_transport(node_handle)
{
  namespace ph = std::placeholders;
  ros::NodeHandle pnh("~");
  // parametrization
  pnh.param<std::string>("camera_base_topic", camera_base_topic,
                         "/camera/color/image_raw");
  pnh.param<std::string>("camera_frame_id", camera_frame_id, "camera");
  pnh.param<std::string>("aruco_base_topic", aruco_base_topic,
                         "/aruco/color/image_raw");
  pnh.param<double>("marker_length", marker_length, 5e-2);
  // pub and sub for the camera
  camera_subscriber = image_transport.subscribeCamera(
      camera_base_topic, 1,
      std::bind(&ArucoPoseNode::process_camera, this, ph::_1, ph::_2));
  aruco_publisher = image_transport.advertiseCamera(aruco_base_topic, 1);
}

void ArucoPoseNode::process_camera(
    const sensor_msgs::ImageConstPtr &image,
    const sensor_msgs::CameraInfoConstPtr &camera_info)
{
  // convert everything
  auto cv_image = cv_bridge::toCvCopy(image);
  auto camera_matrix = cv_camera_matrix(camera_info);
  auto dist_coeffs = cv_dist_coeffs(camera_info);
  // detect the markers and calculate the pose
  auto markers = detect_aruco_markers(cv_image->image);
  auto marker_poses = aruco_poses(markers, camera_matrix, dist_coeffs,
                                  marker_length);
  // publish poses
  std::vector<geometry_msgs::TransformStamped> tf2_poses;
  for (size_t i = 0; i < marker_poses.size(); i++)
  {
    geometry_msgs::TransformStamped tf2_msg;
    tf2_msg.child_frame_id = "aruco_marker_" + std::to_string(markers.ids[i]);
    tf2_msg.transform = convert_to_tf2(marker_poses[i]);
    tf2_msg.header.frame_id = camera_frame_id;
    tf2_msg.header.stamp = image->header.stamp;
    tf2_poses.push_back(tf2_msg);
  }
  tf_broadcast.sendTransform(tf2_poses);
  // draw the poses
  if (aruco_publisher.getNumSubscribers() > 0)
  {
    for (auto pose : marker_poses)
    {
      cv_image->image = draw_pose(cv_image->image, camera_matrix, dist_coeffs,
                                  pose, marker_length);
    }
    aruco_publisher.publish(cv_image->toImageMsg(), camera_info);
  }
}
} // namespace charuco_detect

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_pose_node", ros::init_options::AnonymousName);
  charuco_detect::ArucoPoseNode aruco_pose_node;
  ros::spin();
  return EXIT_SUCCESS;
}