/*
 * @file      tower_robot.hpp
 * @brief     Robot code for tower building
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-03-10
 * @copyright (MIT) 2016 RAD-UoE Informatics
*/

#ifndef TOWER_ROBOT_HPP
#define TOWER_ROBOT_HPP

#include <baxter_cpp/baxter_robot.hpp>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class TowerRobot {
 public:
  explicit TowerRobot(ros::NodeHandle* nh);
  ~TowerRobot();

  void Init();
  void RosSetup();
  void LoadParams();

  void InitRobot();
  void RunDemo();

  bool FindCube();
  bool PickCube();
  bool PlaceCube();

  void TagDetectionsCB(
    const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);
  // void RightRangeCB(const sensor_msgs::Range::ConstPtr& msg);

 private:
  ros::NodeHandle* nh_;
  BaxterRobot blurr_;
  BaxterArm* arm_;
  BaxterRobot::ArmSide arm_side_;
  std::string arm_name_;
  std::string gripper_frame_;
  std::string tower_frame_;
  ros::Rate rate_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber tag_detection_sub_;
  // ros::Subscriber right_range_sub_;

  double pick_x_offset_;
  double pick_y_offset_;
  double pick_z_offset_;

  sensor_msgs::Range right_arm_range_;
  std::map<std::string, int> cam_settings_;

  int target_cube_;
  int stacked_cubes_;
  apriltags_ros::AprilTagDetectionArray tag_array_;
  std::vector<int> cubes_;
  std::map<int, int> cube_status_; // Cube, Stacked/Not Stacked (1/0)
};

#endif /* TOWER_ROBOT_HPP */
