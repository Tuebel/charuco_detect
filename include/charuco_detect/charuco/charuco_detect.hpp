#pragma once
#include <charuco_detect/charuco/board.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <tuple>
#include <vector>

namespace charuco_detect
{
/*!
\brief Detects the boards corner and ids of the board
\param image markers will be detected in this.
\param board to be detected
\returns the updated board with the visible markers
*/
Board detect_board(const cv::Mat &image, Board board);

/*!
\brief Detects and highlights the the charuco boards pose in the given image.
\param board to detect
\param detected_board information about the detected board.
\param camera_matrix K matrix of the camera.
\param dist_coefss D vector of the camera (plumb bob model)
*/
Pose charuco_pose(const Board &board, const cv::Mat &camera_matrix,
                  const cv::Mat &dist_coeffs);
} // namespace charuco_detect
