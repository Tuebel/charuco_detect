#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>

int main(int argc, char **argv)
{
  // Initroialize ROS
  ros::init(argc, argv, "write_marker", ros::init_options::AnonymousName);
  // Private node handle for parameters
  ros::NodeHandle pnh("~");
  // Get the model filename
  std::string filename;
  // Init default if no parameter is supplied
  if (!pnh.getParam("filename", filename))
  {
    filename = ros::package::getPath("charuco_detect") + "/markers/Marker.png";
  }
  int id = 23;
  pnh.param<int>("id", id, 23);
  auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  // Write
  cv::Mat image;
  cv::aruco::drawMarker(dict, id, 200, image);
  // Save
  cv::imwrite(filename, image);
  return EXIT_SUCCESS;
}