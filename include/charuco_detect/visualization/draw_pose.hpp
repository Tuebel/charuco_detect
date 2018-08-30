#pragma once
#include <charuco_detect/pose.hpp>
#include <opencv2/core/core.hpp>

namespace charuco_detect
{
/*!
Draws the pose into the image and returns the result
\param length of the coordiantes in meters
*/
cv::Mat draw_pose(cv::Mat image, const cv::Mat &camera_matrix,
                  const cv::Mat &dist_coeffs, const Pose &pose, float length);
} // namespace charuco_detect