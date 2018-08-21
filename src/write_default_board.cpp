// Charuco
#include "charuco_utils/charuco_utils.h"
// ROS
#include <ros/ros.h>
#include <ros/package.h>
// stl
#include <string>

int main(int argc, char **argv)
{
    // Initroialize ROS
    ros::init(argc, argv, "pcl_model_pub", ros::init_options::AnonymousName);
    // Private node handle for parameters
    ros::NodeHandle pnh("~");
    // Get the model filename
    std::string filename;
    // Init default if no parameter is supplied
    if (!pnh.getParam("filename", filename))
    {
        filename = ros::package::getPath("robo_guide") + "/config/DefaultBoard.png";
    }
    // Write the default board
    auto board = charuco_utils::create_default_board();
    charuco_utils::write_board(filename, board);
    return EXIT_SUCCESS;
}