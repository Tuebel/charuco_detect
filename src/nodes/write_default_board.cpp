#include <charuco_detect/charuco/board.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>

int main(int argc, char **argv)
{
    // Initroialize ROS
    ros::init(argc, argv, "write_default_board", ros::init_options::AnonymousName);
    // Private node handle for parameters
    ros::NodeHandle pnh("~");
    // Get the model filename
    std::string filename;
    // Init default if no parameter is supplied
    if (!pnh.getParam("filename", filename))
    {
        filename = ros::package::getPath("charuco_detect") + "/markers/DefaultBoard.png";
    }
    // Write the default board
    charuco_detect::Board board;
    cv::Mat image;
    board.board->draw(cv::Size(1000, 1400), image);
    // Save
    cv::imwrite(filename, image);
    return EXIT_SUCCESS;
}