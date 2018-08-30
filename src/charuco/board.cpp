#include <charuco_detect/charuco/board.hpp>

namespace charuco_detect
{
Board::Board(int nx, int ny, float square_length, float marker_length)
{
  board = cv::aruco::CharucoBoard::create(
      nx, ny, square_length, marker_length,
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50));
}
} // namespace charuco_detect