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
  rate_(100) {
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
    // cubes_.push_back(i);
    cube_status_[cubes_[i]] = 0;
  }
}

void TowerRobot::RosSetup() {
  //   model_pub_ = nh_->advertise<model_msgs::ModelHypotheses>
  //                (robot_name_ + "/model/hypotheses", 1);
  //   ros::service::waitForService(robot_name_ + "/planner/setup_new_planner");
  //   setup_new_planner_ = nh_->serviceClient<planner_msgs::SetupNewPlanner>
  //                        (robot_name_ + "/planner/setup_new_planner", true);
  tag_detection_sub_ = nh_->subscribe("/tag_detections", 1000,
                                      &TowerRobot::TagDetectionsCB, this);
  right_range_sub_ = nh_->subscribe("/robot/range/right_hand_range/state", 1000,
                                    &TowerRobot::RightRangeCB, this);
}

void TowerRobot::LoadParams() {
  ros::param::get("/baxter_tower/cubes", cubes_);
}

void TowerRobot::InitRobot() {
  blurr_.RightArm().OpenCamera(1280, 800, 30, cam_settings_);
  blurr_.RightArm().EndEffector("release");

  // blurr_.LeftArm().OpenCamera(1280, 800, 30, cam_settings_);
  // blurr_.LeftArm().EndEffector("release");

  ROS_INFO("Calibrating Right Init pose");
  blurr_.CalibrateArmPose(blurr_.kRight, "right_init");
  ROS_INFO("Right Init calibrated!");

  ROS_INFO("Calibrating Above Tower pose");
  blurr_.CalibrateArmPose(blurr_.kRight, "above_tower");
  ROS_INFO("Above Tower calibrated!");
}

void TowerRobot::RunDemo() {
  ROS_INFO("Begin Demo");
  ros::spinOnce();
  blurr_.Head().Pan(0);

  ROS_INFO("Moving to Above Tower");
  blurr_.MoveToPose(blurr_.kRight, "above_tower");
  // blurr_.RightArm().SetInnerLED(true);
  // blurr_.RightArm().SetOuterLED(true);

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
  blurr_.MoveToPose(blurr_.kRight, "above_tower");
  // blurr_.MoveToPose(blurr_.kLeft, "left_init");
  blurr_.RightArm().EndEffector("release");
  // blurr_.LeftArm().EndEffector("release");
}

bool TowerRobot::FindCube() {
  ROS_INFO("Finding Cube...");
  bool success = false;
  blurr_.RightArm().EndEffector("release");

  // blurr_.MoveToPose(blurr_.kRight, "above_tower");

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

      tf2::Transform grasp_pose;
      grasp_pose.getBasis().setRPY(M_PI, 0, -M_PI / 2);
      grasp_pose.setOrigin(tf2::Vector3(0.0, 0.02, 0.0));

      // TODO: Add failure check
      blurr_.MoveToFrame(blurr_.kRight, frame_name.str(), grasp_pose, true);

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

  blurr_.MoveToward(blurr_.kRight, grip_pose);
  blurr_.RightArm().EndEffector("grip");
  blurr_.MoveToPose(blurr_.kRight, "above_tower");

  // Check if picked cube successfully
  (right_arm_range_.range < 0.1) ? success = true : success = false;

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
  place_pose.getBasis().setRPY(0, 0, 0);
  ROS_INFO_STREAM("NoStacked: " << stacked_cubes_);
  place_pose.setOrigin(tf2::Vector3(0.0, 0.0, (0.49 - (stacked_cubes_ * 0.05))));

  blurr_.MoveToward(blurr_.kRight, place_pose);
  blurr_.RightArm().EndEffector("release");
  blurr_.MoveToPose(blurr_.kRight, "above_tower");

  success = true;

  if (success) {
    ROS_INFO("Cube Placed!");
    blurr_.Head().Nod();
    cube_status_[target_cube_] = 1;
    stacked_cubes_ += 1;
    return true;
  }
  return false;
}

void TowerRobot::TagDetectionsCB(
  const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg) {
  tag_array_ = *msg;
}

void TowerRobot::RightRangeCB(const sensor_msgs::Range::ConstPtr& msg) {
  right_arm_range_ = *msg;
}
