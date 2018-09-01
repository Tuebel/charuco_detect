#pragma once
#include <opencv2/core/core.hpp>

namespace charuco_detect
{
struct Pose
{
  /*! Position in cartesian coordinates */
  cv::Vec3d position;
  /*! Vector representation of the orientation in Rodriguez angles */
  cv::Vec3d rotation;
};
} // namespace charuco_detect