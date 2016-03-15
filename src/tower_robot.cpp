/*
 * @file      tower_robot.cpp
 * @brief     Robot code for tower building
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-03-10
 * @copyright (MIT) 2016 RAD-UoE Informatics
*/

#include "baxter_tower/tower_robot.hpp"

TowerRobot::TowerRobot(ros::NodeHandle* nh) : nh_(nh),
  blurr_(*nh_, "blurr", 100),
  rate_(100),
  tf_listener_(tf_buffer_) {
  this->LoadParams();
  this->Init();
  this->RosSetup();
}

TowerRobot::~TowerRobot() {
  ros::param::del("baxter_tower");
}

void TowerRobot::Init() {
  cam_settings_["exposure"] = 80;
  cam_settings_["gain"] = 30;
  stacked_cubes_ = 0;
  for (int i = 1; i <= cubes_.size(); ++i) {
    cube_status_[cubes_[i]] = 0;
  }
  tower_frame_ = "tower";

  if (arm_name_ == "Right") {
    arm_ = &blurr_.RightArm();
    arm_side_ = blurr_.kRight;
    gripper_frame_ = "right_gripper";
  } else {
    arm_ = &blurr_.LeftArm();
    arm_side_ = blurr_.kLeft;
    gripper_frame_ = "left_gripper";
    pick_x_offset_ *= -1;
  }
}

void TowerRobot::RosSetup() {
  tag_detection_sub_ = nh_->subscribe("/tag_detections", 1000,
                                      &TowerRobot::TagDetectionsCB, this);
  // right_range_sub_ = nh_->subscribe("/robot/range/right_hand_range/state", 1000,
  //                                   &TowerRobot::RightRangeCB, this);
}

void TowerRobot::LoadParams() {
  arm_name_ = "None";
  ros::param::get("/baxter_tower/cubes", cubes_);
  ros::param::get("/baxter_tower/arm", arm_name_);
  if (arm_name_ == "None") {
    ROS_ERROR("ERROR: Arm not selected!");
    ros::shutdown();
  } else if ( (arm_name_ != "Left") && (arm_name_ != "Right") ) {
    ROS_ERROR("ERROR: Please select Right or Left arm");
    ros::shutdown();
  }
  ROS_INFO_STREAM(arm_name_ << " arm selected");

  // X-offset accounts for camera offset inside gripper
  // Y-offset accounts for freeing IR sensor, but keeping cube in camera
  // Z-offset accounts for cube depth pick offset
  ros::param::param("/baxter_tower/pick_x_offset", pick_x_offset_, -0.005);
  ros::param::param("/baxter_tower/pick_y_offset", pick_y_offset_, 0.012);
  ros::param::param("/baxter_tower/pick_z_offset", pick_z_offset_, -0.01);
}

void TowerRobot::InitRobot() {
  arm_->OpenCamera(1280, 800, 30, cam_settings_);
  arm_->EndEffector("release");

  ROS_INFO("Calibrating Above Tower pose");
  blurr_.CalibrateArmPose(arm_side_, "above_tower");
  ROS_INFO("Above Tower calibrated!");
}

void TowerRobot::RunDemo() {
  ROS_INFO("Begin Demo");
  ros::spinOnce();
  blurr_.Head().Pan(0);

  ROS_INFO("Moving to Above Tower");
  blurr_.MoveToPose(arm_side_, "above_tower");

  while (ros::ok() && (cubes_.size() > stacked_cubes_)) {

    if (FindCube()) {
      if (PickCube()) {
        PlaceCube();
      }
    }

    ros::spinOnce();
    rate_.sleep();
  }
  ROS_INFO("Tower Assembled!");

  blurr_.Head().Pan(0);
  blurr_.MoveToPose(arm_side_, "above_tower");
  arm_->EndEffector("release");
}

bool TowerRobot::FindCube() {
  ROS_INFO("Finding Cube...");
  bool success = false;
  arm_->EndEffector("release");

  while (ros::ok()) {
    ros::spinOnce();

    int cube_no = -1;
    // Check if any cubes are detected
    if (tag_array_.detections.size() > 0) {
      for (int i = 0; i < tag_array_.detections.size(); ++i) {
        for (int j = 0; j < cubes_.size(); ++j) {
          // Check if the cube is meant to be picked
          if (tag_array_.detections[i].id == cubes_[j]) {
            // Check if the cube has not been stacked
            if (cube_status_[cubes_[j]] == 0) {
              cube_no = cubes_[j];
            }
          }
        }
      }
    }

    // Check if any cube may be picked up
    if (cube_no > -1) {
      target_cube_ = cube_no;
      ROS_INFO_STREAM("TargetCube: " << target_cube_);
      std::stringstream frame_name;
      frame_name << "cube_" << target_cube_;

      tf2::Transform best_pose;
      best_pose.getBasis().setRPY(M_PI, 0, 0);
      best_pose.setOrigin(tf2::Vector3(pick_x_offset_,
                                       pick_y_offset_,
                                       pick_z_offset_));

      best_pose = blurr_.CheckBestApproach(arm_side_, frame_name.str(), best_pose);

      // TODO: Add failure check
      blurr_.MoveToFrame(arm_side_, frame_name.str(), best_pose, true);

      success = true;
      if (success) {
        ROS_INFO_STREAM("Cube_" << target_cube_ << " Found!");
        blurr_.Head().Nod();
        return true;
      }
    }
    rate_.sleep();
  }
  return false;
}

bool TowerRobot::PickCube() {
  bool success;
  ROS_INFO("Picking Cube...");

  tf2::Transform grip_pose;
  grip_pose.getBasis().setRPY(0, 0, 0);
  grip_pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.19));

  blurr_.MoveToward(arm_side_, grip_pose);
  ros::Duration(0.2).sleep();
  arm_->EndEffector("grip");
  blurr_.MoveToPose(arm_side_, "above_tower");

  std::stringstream frame_name;
  frame_name << "cube_" << target_cube_;
  geometry_msgs::TransformStamped grip_cube_tf;
  try {
    grip_cube_tf = tf_buffer_.lookupTransform(gripper_frame_, frame_name.str(),
                                              ros::Time());
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  // Check if picked cube successfully
  // (right_arm_range_.range < 0.1) ? success = true : success = false;
  (grip_cube_tf.transform.translation.z < 0.0) ? success = true : success = false;

  if (success) {
    ROS_INFO_STREAM("Cube_" << target_cube_ << " Picked!");
    blurr_.Head().Nod();
    return true;
  } else {
    ROS_INFO("Cube Missed!");
    return false;
  }
}

bool TowerRobot::PlaceCube() {
  ROS_INFO("Placing Cube...");
  bool success = false;

  tf2::Transform place_pose;
  place_pose.getBasis().setRPY(M_PI, 0, 0);
  place_pose.setOrigin(tf2::Vector3(-0.1, 0, -0.01 + (stacked_cubes_ * 0.05)));

  // TODO: Add failure check
  blurr_.MoveToFrame(arm_side_, tower_frame_, place_pose, true);

  arm_->EndEffector("release");
  ros::Duration(0.2).sleep();

  std::stringstream frame_name;
  frame_name << "cube_" << target_cube_;

  tf2::Transform best_pose;
  best_pose.getBasis().setRPY(M_PI, 0, 0);
  best_pose.setOrigin(tf2::Vector3(pick_x_offset_,
                                   pick_y_offset_,
                                   pick_z_offset_));

  best_pose = blurr_.CheckBestApproach(arm_side_, frame_name.str(), best_pose);

  // TODO: Add failure check
  blurr_.MoveToFrame(arm_side_, frame_name.str(), best_pose, true);

  blurr_.MoveToPose(arm_side_, "above_tower");

  success = true;

  if (success) {
    ROS_INFO("Cube Placed!");
    blurr_.Head().Nod();
    // cube_status_[target_cube_] = 1;
    // stacked_cubes_ += 1;
    return true;
  }
  return false;
}

void TowerRobot::TagDetectionsCB(
  const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg) {
  tag_array_ = *msg;
}

// void TowerRobot::RightRangeCB(const sensor_msgs::Range::ConstPtr& msg) {
//   right_arm_range_ = *msg;
// }
