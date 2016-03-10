/*
 * @file      baxter_tower.cpp
 * @brief     Main baxter_tower executable
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-03-10
 * @copyright (MIT) 2016 RAD-UoE Informatics
*/

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "baxter_tower");
  ros::NodeHandle nh("baxter_tower");

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
