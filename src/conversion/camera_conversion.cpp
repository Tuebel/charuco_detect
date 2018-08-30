#include <charuco_detect/conversion/camera_conversion.hpp>

namespace charuco_detect
{

cv::Mat cv_camera_matrix(const sensor_msgs::CameraInfoConstPtr &info)
{
  cv::Mat result(3, 3, CV_64FC1);
  memcpy(result.data, info->K.data(), 9 * sizeof(double));
  return result;
}

cv::Mat cv_dist_coeffs(const sensor_msgs::CameraInfoConstPtr &info)
{
  cv::Mat result(5, 1, CV_64FC1);
  memcpy(result.data, info->D.data(), 5 * sizeof(double));
  return result;
}
} // namespace charuco_detect