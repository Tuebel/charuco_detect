#pragma once
#include <opencv2/core/core.hpp>

namespace charuco_detect
{
struct Pose
{
  /*! Position in cartesian coordinates */
  cv::Vec3d position;
  /*! Rotation as rotation matrix, Rodriguez angles are not portable */
  cv::Matx33d rotation;
};
} // namespace charuco_detect