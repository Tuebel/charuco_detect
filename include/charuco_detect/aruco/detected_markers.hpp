#pragma once
#include <charuco_detect/pose.hpp>

namespace charuco_detect
{
struct DetectedMarkers
{
  /*! Corners of one marker */
  using Marker = std::vector<cv::Point2f>;
  /*! identification number of the detected markers */
  std::vector<int> ids;
  /*! Pixel coordinates of the marker corners */
  std::vector<Marker> marker_corners;
};
} // namespace charuco_detect