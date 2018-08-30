#pragma once
#include <charuco_detect/aruco/detected_markers.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tuple>

namespace charuco_detect
{
/*!
Detect aruco markers in an OpenCV image.
\return The markers with their ids and corners. The pose is not estimated
*/
DetectedMarkers detect_aruco_markers(const cv::Mat &image);

/*!
Fills in the pose of the markers
\param makers the detected marker corners (from detect_aruco_markers)
\param camera_matrix the calibartion matrix K
\param dist_coeffs the distortion coefficients D
\param marker_length the lenght of one marker in meters
\return the poses of the detected markers
*/
std::vector<Pose> aruco_pose(DetectedMarkers markers,
                                      const cv::Mat &camera_matrix,
                                      const cv::Mat &dist_coeffs,
                                      float marker_length);
} // namespace charuco_detect