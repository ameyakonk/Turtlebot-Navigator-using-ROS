/************************************************************************************
 * BSD 3-Clause License
 * Copyright (c) 2021, Ameya Konkar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    Navigation.cpp
 *  @author  Ameya Konkar (ameyakonk)
 *  @date    11/29/2021
 *  @version 1.0
 *
 *  @brief   Navigation class definition.
 *
 *  @section DESCRIPTION
 *
 *  Source file for Navigation class. Implements turtlebot robot navigation 
 * and obstacle avoidance code
 *
 */

#include "turtlebot_navigator/navigation.hpp"

Navigation::Navigation(ros::NodeHandle &n):n_(n) {
  std::string cmd_vel_topic_name = n_.param<std::string>("cmd_vel_topic_name",
   "");
  cmd_vel_pub_   = n_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  sub = n_.subscribe("scan", 10, &Navigation::laserdata, this);
}

Navigation::~Navigation() {}


void Navigation::laserdata(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // getting laser data for angle=90 degree
  auto angleRes = msg->angle_increment;
  float sensor_data = 90*M_PI/180;

  // if laser data inrange 0.4-0.7, change direction.
  if (msg->ranges[sensor_data] >= 0.4 && msg->ranges[sensor_data] < 0.7) {
    ROS_ERROR_STREAM("Obstacle detected");
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = 0.3;
    cmd_vel.angular.z =  -1;
    cmd_vel_pub_.publish(cmd_vel);
  } else if (msg->ranges[sensor_data] < 0.4) {
    ROS_ERROR_STREAM("Obstacle in close proximity");
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = -0.1;
    cmd_vel.angular.z =  -1;
    cmd_vel_pub_.publish(cmd_vel);
  } else {  // if laser data greater than 0.4, move straight
    ROS_INFO_STREAM("Laser reading: " << msg->ranges[sensor_data]);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = 0.5;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug)) {
  ros::console::notifyLoggerLevelsChanged();
  }
  Navigation N(nh);
  ros::spin();
  return 0;
}
