# charuco_detect
Charuco detection for ROS. Publishes the pose as tf2 and an image for visualization.

Subscirbes to a camera using the image_transport package. Detects the charuco board in the image and publishes its pose via the tf2_ros package. Additionally a visualization of the pose is created using OpenCV.
