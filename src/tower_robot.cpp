/*
 * @file      tower_robot.cpp
 * @brief     Robot code for tower building
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-03-10
 * @copyright (MIT) 2016 RAD-UoE Informatics
*/

#include "baxter_tower/tower_robot.hpp"

#define CLEAR() std::cerr << "\x1B[2J\x1B[H";

TowerRobot::TowerRobot(ros::NodeHandle* nh) : nh_(nh),
  rate_(100),
  tf_listener_(tf_buffer_),
  blurr_(*nh_, "blurr", 100) {
  this->LoadParams();
  this->Init();
  this->RosSetup();
}

TowerRobot::~TowerRobot() {
  ros::param::del("baxter_tower");
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
  ros::param::param("/baxter_tower/exposure", exposure_, 80);
  ros::param::param("/baxter_tower/gain", gain_, 40);
  ros::param::param("/baxter_tower/explore_poses", explore_poses_, 0);
  ros::param::param("/baxter_tower/search_time", search_time_, 5.0);

  // X-offset accounts for freeing IR sensor, but keeping cube in camera
  // Y-offset accounts for camera offset inside gripper
  // Z-offset accounts for cube depth pick offset
  ros::param::param("/baxter_tower/pick_x_offset", pick_x_offset_, -0.005);
  ros::param::param("/baxter_tower/pick_y_offset", pick_y_offset_, 0.0);
  ros::param::param("/baxter_tower/pick_z_offset", pick_z_offset_, -0.01);

  // X,Y,Z offset relative to tower frame (Tower Apriltag)
  ros::param::param("/baxter_tower/place_x_offset", place_x_offset_, -0.1);
  ros::param::param("/baxter_tower/place_y_offset", place_y_offset_, 0.0);
  ros::param::param("/baxter_tower/place_z_offset", place_z_offset_, -0.01);

  // Default Parameters
  tower_frame_ = "tower";
  ros::param::get("/baxter_tower/tower_frame", tower_frame_);
  ros::param::param("/baxter_tower/sleep_time", sleep_time_, 0.2);
  // Z offset positioning gripper above cube for close approach
  ros::param::param("/baxter_tower/find_z_offset", find_z_offset_, -0.01);
  ros::param::param("/baxter_tower/cube_size", cube_size_, 0.05);
  ros::param::param("/baxter_tower/approach_z_offset", approach_z_offset_, 0.15);
  ros::param::param("/baxter_tower/pull_z_offset", pull_z_offset_, -0.05);
}

void TowerRobot::Init() {
  reset_ = false;
  pause_ = false;
  paused_ = false;
  cube_selected_ = grip_calculated_ = pickup_calculated_ = false;
  cam_settings_["exposure"] = exposure_;
  cam_settings_["gain"] = gain_;
  stacked_cubes_ = 0;
  for (int i = 0; i < cubes_.size(); ++i) {
    cube_status_[cubes_[i]] = 0;
    // ROS_INFO_STREAM("Cube: " << cubes_[i] <<
    //                 " Status: " << cube_status_[cubes_[i]]);
  }

  if (arm_name_ == "Right") {
    arm_ = &blurr_.RightArm();
    arm_side_ = blurr_.kRight;
    gripper_frame_ = "right_gripper";
    side_ = "right";
  } else {
    arm_ = &blurr_.LeftArm();
    arm_side_ = blurr_.kLeft;
    gripper_frame_ = "left_gripper";
    pick_y_offset_ *= -1;
    side_ = "left";
  }
}

void TowerRobot::RosSetup() {
  tag_detection_sub_ = nh_->subscribe("/tag_detections", 1000,
                                      &TowerRobot::TagDetectionsCB, this);
  reset_demo_sub_ =
    nh_->subscribe("/robot/digital_io/torso_" + side_ + "_button_back/state",
                   1, &TowerRobot::ResetDemoCB, this);
  pause_demo_sub_ =
    nh_->subscribe("/robot/digital_io/torso_" + side_ + "_button_show/state",
                   1, &TowerRobot::PauseDemoCB, this);
  // right_range_sub_ = nh_->subscribe("/robot/range/right_hand_range/state", 1000,
  //                                   &TowerRobot::RightRangeCB, this);
}

void TowerRobot::InitRobot() {
  arm_->OpenCamera(1280, 800, 30, cam_settings_);
  arm_->EndEffector("release");

  ROS_INFO("Calibrating Above Tower pose");
  blurr_.CalibrateArmPose(arm_side_, "above_tower");
  ROS_INFO("Above Tower calibrated!");

  if (explore_poses_ <= 0) {ROS_WARN("WARNING: No explore poses set!");}
  for (int i = 0; i < explore_poses_; ++i) {
    ROS_INFO_STREAM("Calibrating explore pose " << i);
    std::stringstream explore_pose;
    explore_pose << "explore_" << i;
    blurr_.CalibrateArmPose(arm_side_, explore_pose.str());
    ROS_INFO_STREAM("Explore pose " << i << " calibrated!");
  }
}

void TowerRobot::RunDemo() {
  CLEAR()
  ROS_INFO("Begin Demo");
  ros::spinOnce();
  blurr_.Head().Pan(0);
  blurr_.SetLED("torso_" + side_ + "_outer_light", false);

  ROS_INFO("Moving to Above Tower");
  blurr_.MoveToPose(arm_side_, "above_tower");

  if (reset_) {ROS_INFO("RESET ON");} else {ROS_INFO("RESET OFF");}

  while (ros::ok() && (cubes_.size() > stacked_cubes_) && !reset_) {
    if (FindCube()) {
      if (PickCube()) {
        PlaceCube();
      }
    }
    ros::spinOnce();
    rate_.sleep();
  }

  if (cubes_.size() == stacked_cubes_) {
    ROS_INFO("Tower Assembled!");
  }

  blurr_.Head().Pan(0);
  blurr_.MoveToPose(arm_side_, "above_tower");
  arm_->EndEffector("release");
  arm_->SetOuterLED(false); arm_->SetInnerLED(false);

  ROS_INFO("Waiting for reset...");
  while (ros::ok() && !reset_) {
    ros::spinOnce();
    rate_.sleep();
  }

  if (reset_) {
    ROS_INFO("Resetting!");
    this->LoadParams();
    this->Init();
    this->RosSetup();
    if (reset_) {ROS_INFO("RESET ON");} else {ROS_INFO("RESET OFF");}
  }
}

// ******************** FIND ********************
bool TowerRobot::FindCube() {
  ROS_INFO("Finding Cube...");
  bool success = false;
  arm_->EndEffector("release");
  for (int i = 0; i < cubes_.size(); ++i) {
    ROS_INFO_STREAM("Cube: " << cubes_[i] <<
                    " Status: " << cube_status_[cubes_[i]]);
  }

  double begin = ros::Time::now().toSec();
  int search_pose = 0;
  find_state_ = TowerRobot::find;
  BaxterRobot::Result res;
  cube_selected_ = false;
  int cube_no = -1;
  while (ros::ok() && !success && !reset_) {
    ros::spinOnce();

    if (!pause_) {
      if (paused_) {ROS_WARN("Resume"); paused_ = false;}
      // FIND any cube
      arm_->SetOuterLED(true); arm_->SetInnerLED(true);
      blurr_.SetLED("torso_" + side_ + "_inner_light", false);
      if ( (explore_poses_ > 0) &&
           ( (find_state_ == TowerRobot::find) ||
             (find_state_ == TowerRobot::search) ) ) {
        if ((ros::Time::now().toSec() - begin) > search_time_) {
          if (search_pose >= explore_poses_) {
            search_pose = 0;
          }
          find_state_ = TowerRobot::search;
          std::stringstream explore_pose;
          explore_pose << "explore_" << search_pose;
          res = blurr_.MoveToPoseNB(arm_side_, explore_pose.str(), true);
          if (res == BaxterRobot::success) {
            find_state_ = TowerRobot::find;
            search_pose++;
            begin = ros::Time::now().toSec();
          }
        }
      }

      // Check if any cubes are detected
      if (!cube_selected_) {
        if (tag_array_.detections.size() > 0) {
          for (int i = 0; i < tag_array_.detections.size(); ++i) {
            for (int j = 0; j < cubes_.size(); ++j) {
              // Check if the cube is meant to be picked
              if (tag_array_.detections[i].id == cubes_[j]) {
                // Check if the cube has not been stacked
                if (cube_status_[cubes_[j]] == 0) {
                  cube_no = cubes_[j];
                  ROS_INFO_STREAM("Setting cube " << cube_no);
                }
              }
            }
          }
        }
      }

      // APPROACH detected cube
      if ( (cube_no > -1) &&
           ( (find_state_ == TowerRobot::find) ||
             (find_state_ == TowerRobot::approach) ) ) {
        find_state_ = TowerRobot::approach;

        if (!cube_selected_) {
          ROS_INFO("APPROACH!");
          target_cube_ = cube_no;
          ROS_INFO_STREAM("TargetCube: " << target_cube_);
          target_frame_.str(std::string());  // Clear target_frame
          target_frame_ << "cube_" << target_cube_;
          cube_selected_ = true;

          // geometry_msgs::TransformStamped cube_tf;
          try {
            cube_tf_ = tf_buffer_.lookupTransform(gripper_frame_, target_frame_.str(),
                                                  ros::Time());
          } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
          }
          // ROS_DEBUG_STREAM(cube_tf_.transform);

          // tf2::Quaternion q;
          // tf2::convert(cube_tf_.transform.rotation, q);
          // double roll, pitch, yaw;
          // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          // ROS_INFO("RPY: %f, %f, %f", roll, pitch, yaw);

          // if (pitch > 0.35) {
          // tf2::Transform approach_pose_;
          // approach_pose_.getBasis().setRPY(0, 0, yaw);
          approach_offset_.getBasis().setRPY(0, 0, 0);
          approach_offset_.setOrigin(tf2::Vector3(cube_tf_.transform.translation.x,
                                                  cube_tf_.transform.translation.y,
                                                  cube_tf_.transform.translation.z
                                                  - approach_z_offset_));
          approach_pose_ = blurr_.CalcOffsetPose(arm_side_, approach_offset_);
        }
        // ROS_INFO("Debug");
        res = blurr_.MoveToNB(arm_side_, approach_pose_, true);
        if (res == BaxterRobot::success) {
          find_state_ = TowerRobot::closein;
          ROS_INFO("CLOSEIN!");
        } else if (res == BaxterRobot::error) {
          ROS_INFO("Approach Error!");
          cube_selected_ = false;
        }
        // }
      }

      // CLOSEIN to best pose just above cube
      if ((cube_no > -1) && (find_state_ == TowerRobot::closein)) {
        tf2::Transform best_pose;
        best_pose.getBasis().setRPY(M_PI, 0, 0);
        best_pose.setOrigin(tf2::Vector3(0.0, 0.0, find_z_offset_));

        best_pose = blurr_.CheckBestApproach(arm_side_, target_frame_.str(), best_pose);

        res = blurr_.MoveToFrameNB(arm_side_, target_frame_.str(), best_pose, true);
        if (res == BaxterRobot::success) {
          find_state_ = TowerRobot::find_end;
        }
      }

      if ((cube_no > -1) && (find_state_ == TowerRobot::find_end)) {
        // TODO: Add failure check
        success = true;
        if (success) {
          ROS_INFO_STREAM("Cube_" << target_cube_ << " Found!");
          blurr_.Head().Nod();
        }
      }
    } else {
      arm_->SetOuterLED(false); arm_->SetInnerLED(false);
      blurr_.SetLED("torso_" + side_ + "_inner_light", true);
      if (!paused_) {ROS_WARN("Pause"); paused_ = true;}
    }
    rate_.sleep();
  }
  return success;
}

// ******************** PICK ********************
bool TowerRobot::PickCube() {
  pick_state_ = TowerRobot::grip;
  bool success;
  ROS_INFO("Picking Cube...");
  BaxterRobot::Result res;
  grip_calculated_ = pickup_calculated_ = false;

  while (ros::ok() && (pick_state_ != TowerRobot::pick_end) && !reset_) {
    ros::spinOnce();

    if (!pause_) {
      if (paused_) {ROS_WARN("Resume"); paused_ = false;}
      arm_->SetOuterLED(true); arm_->SetInnerLED(true);
      blurr_.SetLED("torso_" + side_ + "_inner_light", false);

      // GRIP the cube found
      if (pick_state_ == TowerRobot::grip) {
        if (grip_calculated_ == false) {
          grip_offset_.getBasis().setRPY(0, 0, 0);
          grip_offset_.setOrigin(tf2::Vector3(pick_x_offset_,
                                              pick_y_offset_,
                                              pick_z_offset_));

          grip_pose_ = blurr_.CalcOffsetPose(arm_side_, grip_offset_);
          grip_calculated_ = true;
        }

        res = blurr_.MoveToNB(arm_side_, grip_pose_);
        if (res == BaxterRobot::success) {
          ros::Duration(sleep_time_).sleep();
          arm_->EndEffector("grip");
          ROS_INFO("Gripped!");
          pick_state_ = TowerRobot::pickup;
        } else if (res == BaxterRobot::error) {
          ROS_INFO("Grip Error!");
          grip_calculated_ = false;
        }
      }

      // PICKUP the gripped cube
      if (pick_state_ == TowerRobot::pickup) {
        if (pickup_calculated_ == false) {
          pickup_offset_.getBasis().setRPY(0, 0, 0);
          pickup_offset_.setOrigin(tf2::Vector3(0.0, 0.0, pull_z_offset_));

          pickup_pose_ = blurr_.CalcOffsetPose(arm_side_, pickup_offset_);
          pickup_calculated_ = true;
        }

        res = blurr_.MoveToNB(arm_side_, pickup_pose_);
        if (res == BaxterRobot::success) {
          ROS_INFO("PickedUp!");
          pick_state_ = TowerRobot::check;
        } else if (res == BaxterRobot::error) {
          ROS_INFO("PickUp Error!");
          pickup_calculated_ = false;
        }
      }

      if (pick_state_ == TowerRobot::check) {
        geometry_msgs::TransformStamped grip_cube_tf;
        try {
          grip_cube_tf = tf_buffer_.lookupTransform(gripper_frame_, target_frame_.str(),
                                                    ros::Time());
        } catch (tf2::TransformException& ex) {
          ROS_ERROR("%s", ex.what());
        }

        ROS_DEBUG_STREAM("T: " << ros::Time::now().toSec() -
                         grip_cube_tf.header.stamp.toSec());
        ROS_DEBUG_STREAM("Z: " << grip_cube_tf.transform.translation.z);
        // Check if picked cube successfully (Still seen and within grasp)
        // (right_arm_range_.range < 0.1) ? success = true : success = false;
        if ((grip_cube_tf.transform.translation.z < 0.0) &&
            ((ros::Time::now().toSec() - grip_cube_tf.header.stamp.toSec()) < 0.5)) {
          success = true;
        } else {success = false;}
        pick_state_ = TowerRobot::pick_end;
      }

      if (pick_state_ == TowerRobot::pick_end) {
        if (success) {
          ROS_INFO_STREAM("Cube_" << target_cube_ << " Picked!");
          blurr_.Head().Nod();
        } else {
          ROS_INFO("Cube Missed!");
        }
      }
    } else {
      arm_->SetOuterLED(false); arm_->SetInnerLED(false);
      blurr_.SetLED("torso_" + side_ + "_inner_light", true);
      if (!paused_) {ROS_WARN("Pause"); paused_ = true;}
    }
    rate_.sleep();
  }
  return success;
}

// ******************** PLACE ********************
bool TowerRobot::PlaceCube() {
  ROS_INFO("Placing Cube...");
  place_state_ = TowerRobot::above;
  bool success = false;
  BaxterRobot::Result res;
  // grip_calculated_ = pickup_calculated_ = false;

  while (ros::ok() && (place_state_ != TowerRobot::place_end) && !reset_) {
    ros::spinOnce();

    if (!pause_) {
      if (paused_) {ROS_WARN("Resume"); paused_ = false;}
      arm_->SetOuterLED(true); arm_->SetInnerLED(true);
      blurr_.SetLED("torso_" + side_ + "_inner_light", false);

      // Go ABOVE tower for good positioning
      if (place_state_ == TowerRobot::above) {
        res = blurr_.MoveToPoseNB(arm_side_, "above_tower");
        if (res == BaxterRobot::success) {
          place_state_ = TowerRobot::place;
        }
      }

      // Final PLACE and release of cube
      if (place_state_ == TowerRobot::place) {
        place_offset_.getBasis().setRPY(M_PI, 0, 0);
        place_offset_.setOrigin(tf2::Vector3(place_x_offset_, place_y_offset_,
                                             place_z_offset_ +
                                             (stacked_cubes_ * cube_size_)));

        // TODO: Add failure check
        res = blurr_.MoveToFrameNB(arm_side_, tower_frame_, place_offset_, true);
        if (res == BaxterRobot::success) {
          place_state_ = TowerRobot::retract;
          arm_->EndEffector("release");
          ros::Duration(sleep_time_).sleep();
        }
      }

      // RETRACT once cube is released
      if (place_state_ == TowerRobot::retract) {
        tf2::Transform best_pose;
        best_pose.getBasis().setRPY(M_PI, 0, 0);
        best_pose.setOrigin(tf2::Vector3(0.0, 0.0, find_z_offset_));

        best_pose = blurr_.CheckBestApproach(arm_side_, target_frame_.str(), best_pose);

        // TODO: Add failure check
        res = blurr_.MoveToFrameNB(arm_side_, target_frame_.str(), best_pose, true);
        if (res == BaxterRobot::success) {
          place_state_ = TowerRobot::reabove;
        }
      }

      // Go BACK ABOVE the tower for safety
      if (place_state_ == TowerRobot::reabove) {
        res = blurr_.MoveToPoseNB(arm_side_, "above_tower");
        if (res == BaxterRobot::success) {
          place_state_ = TowerRobot::place_end;
          success = true;
        }
      }

      if (place_state_ == TowerRobot::place_end) {
        if (success) {
          ROS_INFO("Cube Placed!");
          blurr_.Head().Nod();
          cube_status_[target_cube_] = 1;
          stacked_cubes_ += 1;
          for (int i = 0; i < cubes_.size(); ++i) {
            // ROS_INFO_STREAM("Cube: " << cubes_[i] <<
            //                 " Status: " << cube_status_[cubes_[i]]);
          }
        } else {
          ROS_INFO("Cube Place Failed!");
        }
      }
    } else {
      arm_->SetOuterLED(false); arm_->SetInnerLED(false);
      blurr_.SetLED("torso_" + side_ + "_inner_light", true);
      if (!paused_) {ROS_WARN("Pause"); paused_ = true;}
    }
    rate_.sleep();
  }
  return success;
}

void TowerRobot::TagDetectionsCB(
  const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg) {
  tag_array_ = *msg;
}

// void TowerRobot::RightRangeCB(const sensor_msgs::Range::ConstPtr& msg) {
//   right_arm_range_ = *msg;
// }
