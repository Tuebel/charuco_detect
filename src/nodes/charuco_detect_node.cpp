#include <charuco_detect/charuco/charuco_detect.hpp>
#include <charuco_detect/conversion/pose_conversion.hpp>
#include <charuco_detect/visualization/draw_pose.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

/*!
\brief Subscribes to a camera and publishes the charuco image and pose.
*/
class CharucoPosePublisher
{
private:
  // ROS pub and sub
  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_transport;
  image_transport::CameraSubscriber cam_sub;
  image_transport::CameraPublisher cam_pub;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  // Board to detect
  charuco_detect::Board board;
  // Configuration
  std::string camera_frame;
  std::string charuco_frame;

public:
  CharucoPosePublisher() : image_transport(node_handle)
  {
    // Init parameters
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("camera_frame", camera_frame, "camera");
    pnh.param<std::string>("charuco_frame", charuco_frame, "charuco");
    // Subscriber and publisher for the camera
    cam_sub = image_transport.subscribeCamera(
        "/camera/color/image_raw", 1, &CharucoPosePublisher::camera_callback,
        this);
    cam_pub = image_transport.advertiseCamera("/charuco/image", 1);
  }

  void camera_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
  {
    // Convert the ros image to opencv
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(image_msg);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Extract camera calibration
    cv::Mat camera_matrix(3, 3, CV_64F, (void *)info_msg->K.data());
    cv::Mat distortion_coeffs(5, 1, CV_64F, (void *)info_msg->D.data());
    // Detect the charuco board and pose
    board = charuco_detect::detect_board(cv_ptr->image, board);
    // Estimate the pose
    auto estimated_pose = charuco_detect::charuco_pose(board, camera_matrix,
                                                       distortion_coeffs);
    // Create the transform message
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = image_msg->header.stamp;
    tf_stamped.header.frame_id = camera_frame;
    tf_stamped.child_frame_id = charuco_frame;
    tf_stamped.transform = charuco_detect::convert_to_tf2(estimated_pose);
    // Publish the pose
    tf_broadcaster.sendTransform(tf_stamped);
    // Modify the image if subscribers are available
    if (cam_pub.getNumSubscribers() > 0)
    {
      charuco_detect::draw_pose(cv_ptr->image, camera_matrix, distortion_coeffs,
                                estimated_pose, board.board->getMarkerLength());
    }
    // Always output camera if subscribers are available.
    // Might not be modified.
    if (cam_pub.getNumSubscribers() > 0)
    {
      cam_pub.publish(cv_ptr->toImageMsg(), info_msg);
    }
  }
};

int main(int argc, char **argv)
{
  // Init ROS
  ros::init(argc, argv, "charuco_detect_node", ros::init_options::AnonymousName);
  // Run the publisher
  CharucoPosePublisher pub;
  ros::spin();
  return EXIT_SUCCESS;
}
