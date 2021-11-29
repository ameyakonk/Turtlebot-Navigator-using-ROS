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
 *  @file    Navigation.hpp
 *  @author  Ameya Konkar (ameyakonk)
 *  @date    11/29/2021
 *  @version 1.0
 *
 *  @brief   Navigation class declaration.
 *
 *  @section DESCRIPTION
 *
 *  Header file for Navigation class. Creates function definition for
 *  Navigation class.
 *
 */

#ifndef INCLUDE_TURTLEBOT_NAVIGATOR_NAVIGATION_HPP_
#define INCLUDE_TURTLEBOT_NAVIGATOR_NAVIGATION_HPP_

// ROS headers
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

// CPP Headers
#include <stdlib.h>
#include <sstream>
#include <cmath>

// ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"




/**
 *  @brief Class Navigation
 *
 *  The following class Navigation publishes linear and angular velocity to a
 *  turtlebot based on laser scan data to perform obstacle avoidance in a given
 *  world.
 */
class Navigation {
 private:
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber sub;

 public:
        ros::NodeHandle n_;
		/**
		   *   @brief  Default constructor for Navigation. Sets up a publisher topic with
		   *           ROS master and subscribes to laser scan data
		   *
		   *   @param  n_ as ROS nodehandle object
		   *
		   *   @return void
		   */
        explicit Navigation(ros::NodeHandle&);

		/**
		   *   @brief  Default Destructor for Navigation. 
		   *
		   *   @return void
		   */
        ~Navigation();

		/**
		   *   @brief  gets laser data from the scan topic 
		   *
		   *   @param  msg as sensor_msgs object
		   *
		   *   @return void
		   */
        void laserdata(const sensor_msgs::LaserScan::ConstPtr &msg);
};
#endif  // INCLUDE_TURTLEBOT_NAVIGATOR_NAVIGATION_HPP_
