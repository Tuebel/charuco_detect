#pragma once
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>

namespace charuco_detect
{
/*!
Copies the camera matrix into a cv Mat
*/
cv::Mat cv_camera_matrix(const sensor_msgs::CameraInfoConstPtr &info);

/*!
Copies the distortion coefficients into a cv Mat
*/
cv::Mat cv_dist_coeffs(const sensor_msgs::CameraInfoConstPtr &info);
} // namespace charuco_detect