/*!
\file charuco_detect_node.cpp
\author My Self
\brief Publishes the pose of the charuco board.

Subcribes to /camera/image_raw topic and tries to find the charuco board.
Also subcribes to camera/camera_info to estimate the pose via the camera parameters.
The default namespace is camera
If the board is found a tf2 is broadcasted from camera_frame to charuco_frame
Additionally a visualization is drawn into the frame and published to "charuco/image"
*/

// Charuco utitilies
#include "charuco_utils/charuco_utils.h"
// Ros
#include <ros/ros.h>
// Image processing
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// Transformations
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/*!
\brief Subscribes to a camera and publishes the charuco image and pose.
*/
class CharucoPosePublisher
{
private:
  // ROS pub and sub
  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_trans;
  image_transport::CameraSubscriber cam_sub;
  image_transport::CameraPublisher cam_pub;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  // Board to detect
  charuco_utils::charuco_board charuco_board;
  // Configuration
  std::string camera_frame;
  std::string charuco_frame;

  /*!
    \brief Creates the tf2 transformation form opencv rvec and tvec
    */
  geometry_msgs::Transform create_transform(const charuco_utils::estimated_pose &estimated_pose)
  {
    geometry_msgs::Transform transform;
    // Convert to tf2 rotation matrix
    tf2::Matrix3x3 tf_mat;
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        tf_mat[r][c] = estimated_pose.r_mat(r, c);
      }
    }
    // Set quaternions of transform
    tf2::Quaternion q;
    tf_mat.getRotation(q);
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    // Translation
    transform.translation.x = estimated_pose.t_vec[0];
    transform.translation.y = estimated_pose.t_vec[1];
    transform.translation.z = estimated_pose.t_vec[2];

    return transform;
  }

public:
  CharucoPosePublisher() : image_trans(node_handle)
  {
    // Init parameters
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("camera_frame", camera_frame, "camera");
    pnh.param<std::string>("charuco_frame", charuco_frame, "charuco");
    // Create the board to track
    charuco_board = charuco_utils::create_default_board();
    // Subscriber and publisher for the camera
    cam_sub = image_trans.subscribeCamera(
        "/camera/color/image_raw", 1, &CharucoPosePublisher::camera_callback, this);
    cam_pub = image_trans.advertiseCamera("/charuco/image", 1);
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
    auto optional_board = charuco_utils::detect_board(cv_ptr->image, charuco_board);
    if (optional_board)
    {
      auto detected_board = *optional_board;
      auto estimated_pose = charuco_utils::get_charuco_pose(charuco_board,
                                                            detected_board,
                                                            camera_matrix,
                                                            distortion_coeffs);
      if (estimated_pose.valid)
      {
        // Create the transform message
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = camera_frame;
        tf_stamped.child_frame_id = charuco_frame;
        tf_stamped.transform = create_transform(estimated_pose);
        // Publish the pose
        tf_broadcaster.sendTransform(tf_stamped);
        // Modify the image if subscribers are available
        if (cam_pub.getNumSubscribers() > 0)
        {
          charuco_utils::draw_board(cv_ptr->image, detected_board);
          charuco_utils::draw_pose(cv_ptr->image,
                                   camera_matrix,
                                   distortion_coeffs,
                                   estimated_pose,
                                   charuco_board);
        }
      }
      else
      {
        ROS_DEBUG("Could not estimate a valid pose of the charuco board.");
      }
    }
    else
    {
      ROS_DEBUG("No charuco board detected.");
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
