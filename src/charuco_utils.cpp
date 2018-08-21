#include "charuco_utils/charuco_utils.h"

namespace charuco_utils
{

charuco_board create_board(
    int nx,
    int ny,
    float square_length,
    float marker_length)
{
    charuco_board b;
    b.dictionary = ar::getPredefinedDictionary(ar::DICT_6X6_50);
    b.board = ar::CharucoBoard::create(nx, ny, square_length, marker_length, b.dictionary);
    return b;
}

charuco_board create_default_board()
{
    return create_board(5, 7, 0.036, 0.018);
}

void write_board(std::string filename, charuco_board charuco_board)
{
    // Draw board
    cv::Mat image;
    charuco_board.board->draw(cv::Size(1000, 1400), image);
    // Save
    cv::imwrite(filename, image);
}

detected_markers detect_markers(cv::InputArray image, charuco_board charuco_board)
{
    // Detect
    detected_markers dm;
    ar::detectMarkers(image, charuco_board.dictionary, dm.corners, dm.ids);
    return dm;
}

void draw_markers(cv::InputOutputArray image, detected_markers detect_markers)
{
    // Draw if available
    if (detect_markers.ids.size() > 0)
    {
        ar::drawDetectedMarkers(image, detect_markers.corners, detect_markers.ids);
    }
}

boost::optional<detected_board> detect_board(cv::InputArray image, charuco_board charuco_board)
{
    // Find the markers of the board
    auto markers = detect_markers(image, charuco_board);
    // Detect charuco board
    if (markers.ids.size() > 0)
    {
        // Init charuco return values
        detected_board detected_board;
        cv::aruco::interpolateCornersCharuco(markers.corners,
                                             markers.ids,
                                             image,
                                             charuco_board.board,
                                             detected_board.corners,
                                             detected_board.ids);
        return detected_board;
    }
    return boost::none;
}

void draw_board(cv::InputOutputArray image, detected_board detected_board)
{
    // Draw only if available
    if (detected_board.ids.size() > 0)
    {
        ar::drawDetectedCornersCharuco(image, detected_board.corners, detected_board.ids);
    }
}

estimated_pose get_charuco_pose(
    charuco_board charuco_board,
    detected_board detected_board,
    cv::InputArray camera_matrix,
    cv::InputArray dist_coeffs)
{
    // Create the return values
    estimated_pose estimated_pose;
    estimated_pose.valid = false;
    // Estimate pose
    if (detected_board.ids.size() > 0)
    {
        estimated_pose.valid = ar::estimatePoseCharucoBoard(detected_board.corners,
                                                            detected_board.ids,
                                                            charuco_board.board,
                                                            camera_matrix,
                                                            dist_coeffs,
                                                            estimated_pose.r_vec,
                                                            estimated_pose.t_vec);
        if (estimated_pose.valid)
        {
            // Calculate the rotation matrix
            cv::Rodrigues(estimated_pose.r_vec, estimated_pose.r_mat);
        }
    }
    return estimated_pose;
}

void draw_pose(cv::InputOutputArray image,
               cv::InputArray camera_matrix,
               cv::InputArray dist_coeffs,
               const estimated_pose &estimated_pose,
               charuco_board charuco_board)
{
    if (estimated_pose.valid)
    {
        // Draw the axis
        ar::drawAxis(image,
                     camera_matrix,
                     dist_coeffs,
                     estimated_pose.r_vec,
                     estimated_pose.t_vec,
                     charuco_board.board->getSquareLength());
    }
}

} // namespace charuco_utils
