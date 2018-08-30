#pragma once
#include <charuco_detect/aruco/detected_markers.hpp>

namespace charuco_detect
{
/*!
\brief Draws the markers into an image.
\param image The image to draw the corners in.
\param detected_markers the markers to be drawn.
*/
cv::Mat draw_markers(cv::Mat image, DetectedMarkers detected_markers);
} // namespace charuco_detect