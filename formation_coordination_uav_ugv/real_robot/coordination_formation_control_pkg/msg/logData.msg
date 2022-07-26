std_msgs/Header header # time
geometry_msgs/Pose[] current_pose
geometry_msgs/Twist[] current_twist
geometry_msgs/Twist[] current_acc
geometry_msgs/Pose2D command_pose
geometry_msgs/Twist command_twist
geometry_msgs/Pose2D input
geometry_msgs/Vector3 input_formation
geometry_msgs/Vector3 input_orientation
geometry_msgs/Vector3 input_navigation
geometry_msgs/Vector3 input_obstacle
geometry_msgs/Vector3 input_integration
geometry_msgs/Vector3 input_to_system
geometry_msgs/Vector3 input_before_to_system
float32 yaw
float32[] current_obstacle
geometry_msgs/Pose[] ugv_current_pose
geometry_msgs/Pose[] ugv_cluster
