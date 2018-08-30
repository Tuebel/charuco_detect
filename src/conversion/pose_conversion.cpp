#include <charuco_detect/conversion/pose_conversion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace charuco_detect
{
geometry_msgs::Transform convert_to_tf2(const Pose &pose)
{
  tf2::Vector3 position(pose.position[0], pose.position[1],
                        pose.position[2]);
  tf2::Matrix3x3 rotation;
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      rotation[r][c] = pose.rotation(r, c);
    }
  }
  return tf2::toMsg(tf2::Transform(rotation, position));
}
} // namespace charuco_detect