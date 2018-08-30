#include <charuco_detect/aruco/aruco_detect.hpp>
#include <cv_bridge/cv_bridge.h>

namespace charuco_detect
{
DetectedMarkers detect_aruco_markers(const cv::Mat &image)
{
  // find these markers
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  // output of the detection
  DetectedMarkers result;
  cv::aruco::detectMarkers(image, dictionary, result.marker_corners,
                           result.ids);
  return result;
}

std::vector<Pose> aruco_pose(DetectedMarkers markers,
                                const cv::Mat &camera_matrix,
                                const cv::Mat &dist_coeffs,
                                float marker_length)
{
  // estimate the poses
  std::vector<cv::Vec3d> tvecs;
  std::vector<cv::Matx33d> rmats;
  cv::aruco::estimatePoseSingleMarkers(markers.marker_corners, marker_length,
                                       camera_matrix, dist_coeffs, rmats,
                                       tvecs);
  // assemble the results
  std::vector<Pose> poses;
  for (size_t i = 0; i < tvecs.size(); i++)
  {
    Pose pose;
    pose.position = tvecs[i];
    pose.rotation = rmats[i];
    poses.push_back(pose);
  }
  return poses;
}
} // namespace charuco_detect
