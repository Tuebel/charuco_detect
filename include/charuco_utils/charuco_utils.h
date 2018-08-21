#pragma once

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
// stl
#include <tuple>
#include <vector>
// boost
#include <boost/optional.hpp>

// Shorter namespace
namespace ar = cv::aruco;

namespace charuco_utils
{

//! Definition of a baord with its markers.
struct charuco_board
{
    //! The aruco markers inside the white squares.
    cv::Ptr<ar::Dictionary> dictionary;
    //! The board definition (size, number of squares, ...)
    cv::Ptr<ar::CharucoBoard> board;
};

//! Results of a marker detection
struct detected_markers
{
    //! The corners of the aruco markers (n*4 elements).
    std::vector<std::vector<cv::Point2f>> corners;
    //! The ids of the n marker corners
    std::vector<int> ids;
};

//! Results of a charuco board detection
struct detected_board
{
    //! Interpolated corners of the chessboard
    std::vector<cv::Point2f> corners;
    //! The ids of the chessboard corners
    std::vector<int> ids;
};

//! Result of a pose estimation
struct estimated_pose
{
    //! If a valid pose could be estimated
    bool valid;
    //! The rodrigues angles
    cv::Vec3d r_vec;
    //! The rotation matrix
    cv::Matx33d r_mat;
    //! The translation vector
    cv::Vec3d t_vec;
};

/*!
\brief Creates a charuco board and its dictionary of aruco markers.
\param nx the number of markers in x direction.
\param ny the number of markers in y direction.
\param square_length the edge length of the cheessboard.
\param marker_length the edge lenfth of the aruco markers.
\returns the charuco board struct.
*/
charuco_board create_board(
    int nx,
    int ny,
    float square_length,
    float marker_length);

/*!
\brief Creates a 5x7 Charuco board
\returns the charuco board struct.
*/
charuco_board create_default_board();

/*!
\brief Writes the board to a file.
*/
void write_board(std::string filename, charuco_board charuco_board);

/*!
\brief Detects and highlights the aruco markers in the given image.
\param charuco_board detect the markers of this board.
\returns the location of the marker corners(0) and the corresponding marker ids(1).
*/
detected_markers detect_markers(cv::InputArray image, charuco_board charuco_board);

/*!
\brief Draws the markers into an image.
\param image The image to draw the corners in.
\param detected_markers the markers to be drawn.
*/
void draw_markers(cv::InputOutputArray image, detected_markers detected_markers);

/*!
\brief Detects the boards corner and ids of the board
\param image markers will be detected in this.
\param charuco_board the board to be detected
\returns the struct which contains the corners and ids that have been detected.
*/
boost::optional<detected_board> detect_board(cv::InputArray image, charuco_board charuco_board);

/*!
\brief Draws the corners of the charuco board.
\param image The image to draw the corners in.
\param detected_board the board to be drawn.
*/
void draw_board(cv::InputOutputArray image, detected_board detected_board);

/*!
\brief Detects and highlights the the charuco boards pose in the given image.
\param charuco_board the board definition.
\param detected_board information about the detected board.
\param camera_matrix K matrix of the camera.
\param dist_coefss D vector of the camera (plumb bob model)
*/
estimated_pose get_charuco_pose(
    charuco_board charuco_board,
    detected_board detected_board,
    cv::InputArray camera_matrix,
    cv::InputArray dist_coeffs);

void draw_pose(cv::InputOutputArray image,
               cv::InputArray camera_matrix,
               cv::InputArray dist_coeffs,
               const estimated_pose &estimated_pose,
               charuco_board charuco_board);

} // namespace charuco_utils
