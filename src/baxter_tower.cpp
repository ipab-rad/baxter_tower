/*
 * @file      baxter_tower.cpp
 * @brief     Main baxter_tower executable
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-03-10
 * @copyright (MIT) 2016 RAD-UoE Informatics
*/

#include <ros/ros.h>

#include <baxter_tower/tower_robot.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "baxter_tower");
  ros::NodeHandle nh("baxter_tower");
  TowerRobot tower_robot(&nh);

  ros::Rate r(100);

  while (ros::ok()) {
    tower_robot.InitRobot();
    tower_robot.RunDemo();
  }

  ros::shutdown();

  return 0;
}
