/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD
License.] Modified BSD License: Redistribution and use in source and binary forms,with
or without modification,are permitted provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of
conditions and the following disclaimer. *Redistributions in binary form must reproduce
the above copyright notice,this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution. *Neither the names
of the University of Texas at Austin,nor Austin Robot Technology,nor the names of other
contributors maybe used to endorse or promote products derived from this software
without specific prior written permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGH
THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,INCLUDING,BUT
NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL
DAMAGES(INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS
OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE OR
OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE
POSSIBILITY OF SUCHDAMAGE.
************************************************************************************************/

/*
 * File: hesai_ros_driver_node.cc
 * Author: Zhang Yu <zhangyu@hesaitech.com>, Yun Chang <yun@minoic.ai>
 * Description: Hesai sdk node for CPU
 * Created on June 12, 2023, 10:46 AM, Modified on Jul 23, 2025.
 */

#include <signal.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "Version.h"
#include "manager/node_manager.h"

#ifdef ROS_FOUND
#include <ros/ros.h>
#elif defined(ROS2_FOUND)
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#endif

namespace {
bool sig_recv = false;

#ifdef ROS2_FOUND
std::mutex g_mtx;
std::condition_variable g_cv;
#endif

void sigHandler(int sig) {
  sig_recv = true;
#ifdef ROS_FOUND
  ros::shutdown();
#elif defined(ROS2_FOUND)
  g_cv.notify_all();
#endif
}

std::string getConfigPath() {
#ifdef ROS_FOUND
  ros::NodeHandle priv_nh("~");
  std::string config_path;
  priv_nh.param<std::string>("config_path", config_path, "");
  return config_path;
#elif defined(ROS2_FOUND)
  auto node = rclcpp::Node::make_shared("hesai_ros_driver_node");
  return node->declare_parameter<std::string>("config_path", "");
#else
  return "";
#endif
}
}  // namespace

int main(int argc, char** argv) {
  std::cout << "-------- Hesai Lidar ROS V" << VERSION_MAJOR << "." << VERSION_MINOR
            << "." << VERSION_TINY << " --------" << std::endl;

  signal(SIGINT, sigHandler);

#ifdef ROS_FOUND
  ros::init(argc, argv, "hesai_ros_driver_node", ros::init_options::NoSigintHandler);
#elif defined(ROS2_FOUND)
  rclcpp::init(argc, argv);
#endif

  std::string config_path = getConfigPath();
  if (config_path.empty()) {
    std::cerr << "[ERROR] config_path parameter is required." << std::endl;
    return EXIT_FAILURE;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (const YAML::Exception& e) {
    std::cerr << "[ERROR] Failed to load config file: " << config_path << "\n"
              << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  auto node_manager = std::make_shared<NodeManager>();
  node_manager->Init(config);
  node_manager->Start();

  while (!node_manager->IsPlayEnded() && !sig_recv) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  node_manager->Stop();

#ifdef ROS2_FOUND
  rclcpp::shutdown();
#endif

  return 0;
}
