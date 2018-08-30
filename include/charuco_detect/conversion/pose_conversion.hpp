#pragma once
#include <charuco_detect/pose.hpp>
#include <geometry_msgs/Transform.h>

namespace charuco_detect
{
/*!
Convert the pose of the marker to a tf2 transform.
*/
geometry_msgs::Transform convert_to_tf2(const Pose &pose);
} // namespace charuco_detect