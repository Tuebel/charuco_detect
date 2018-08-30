#pragma once
#include <charuco_detect/pose.hpp>
#include <opencv2/aruco/charuco.hpp>

namespace charuco_detect
{
/*!
Stores a charuco board with additional information
*/
struct Board
{
  /*! The board definition */
  cv::Ptr<cv::aruco::CharucoBoard> board;
  /*! Detected chessboard corners of the last frame */
  std::vector<cv::Point2f> corners;
  /*! ids of the detected corners */
  std::vector<int> ids;
  /*! Pose of the marker */
  Pose pose;

  /*! Create the default board with the given dimension
  \param nx number of squares in x-direction
  \param ny number of squares in y-direction
  \param square_length of one chess board square
  \param marker_length of one marker inside the chessboard
  */
  Board(int nx = 5, int ny = 7,
        float square_length = 0.036, float marker_length = 0.018);
};
} // namespace charuco_detect