#include <charuco_detect/aruco/aruco_detect.hpp>
#include <charuco_detect/charuco/charuco_detect.hpp>

namespace charuco_detect
{

Board detect_board(const cv::Mat &image, Board board)
{
  // Find the markers of the board
  auto markers = detect_aruco_markers(image);
  // Detect charuco board
  if (markers.ids.size() > 0)
  {
    // Update board
    cv::aruco::interpolateCornersCharuco(
        markers.marker_corners, markers.ids, image, board.board, board.corners,
        board.ids);
  }
  return board;
}

Pose charuco_pose(const Board &board, const cv::Mat &camera_matrix,
                  const cv::Mat &dist_coeffs)
{
  Pose result;
  // Estimate pose
  if (board.ids.size() > 0)
  {
    cv::aruco::estimatePoseCharucoBoard(board.corners, board.ids, board.board,
                                        camera_matrix, dist_coeffs,
                                        result.rotation,
                                        result.position);
  }
  return result;
}
} // namespace charuco_detect
