#include <charuco_detect/visualization/draw_markers.hpp>
#include <opencv2/aruco.hpp>

namespace charuco_detect
{
cv::Mat draw_markers(cv::Mat image, DetectedMarkers detected_markers)
{
  // Draw if available
  if (detected_markers.ids.size() > 0)
  {
    cv::aruco::drawDetectedMarkers(image, detected_markers.marker_corners,
                                   detected_markers.ids);
  }
  return image;
}
} // namespace charuco_detect