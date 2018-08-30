#include <charuco_detect/visualization/draw_pose.hpp>
#include <opencv2/aruco.hpp>

namespace charuco_detect
{
cv::Mat draw_pose(cv::Mat image,
                  const cv::Mat &camera_matrix,
                  const cv::Mat &dist_coeffs,
                  const Pose &pose,
                  float length)
{
  // Draw the axis
  cv::aruco::drawAxis(image, camera_matrix, dist_coeffs,
                      pose.rotation, pose.position, length);
  return image;
}

} // namespace charuco_detect
