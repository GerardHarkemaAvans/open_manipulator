/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Gerard Harkema */

#ifndef OPEN_MANIPULATOR_FREEDRIVE_H
#define OPEN_MANIPULATOR_FREEDRIVE_H

#include <ros/ros.h>
#include <termios.h>
#include <sys/ioctl.h>

#include "open_manipulator_msgs/SetActuatorState.h"
#include "sensor_msgs/JointState.h"

#define NUM_OF_JOINT_AND_TOOL 5
#define IDLE                0
#define ENABLE_FREEDRIVE    1
#define DISABLE_FREEDRIVE    2
#define EXIT                3


class OpenManipulatorFreedrive
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::Subscriber open_manipulator_joint_states_sub_;

  ros::ServiceClient set_actuator_state_client_;

  std::vector<double> present_joint_angle_;
  std::vector<std::string> joint_name_;

  uint8_t mode_state_;
  bool  abort_;

 public:
  OpenManipulatorFreedrive();
  ~OpenManipulatorFreedrive();

  void initServiceClient();
  void initSubscribe();

  bool setActuatorState(bool enable);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  void publishCallback(const ros::TimerEvent&);
  void setModeState(char ch);
  void demoSequence();

  void printText();
  bool kbhit();
  bool abort();
};

#endif //OPEN_MANIPULATOR_FREEDRIVE_H
