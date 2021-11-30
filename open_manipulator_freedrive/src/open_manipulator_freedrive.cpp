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

#include "open_manipulator_freedrive/open_manipulator_freedrive.h"

OpenManipulatorFreedrive::OpenManipulatorFreedrive()
: node_handle_(""),
  priv_node_handle_("~"),
  mode_state_(IDLE),
  abort_(false)
{
  present_joint_angle_.resize(NUM_OF_JOINT_AND_TOOL, 0.0);

  joint_name_.push_back("joint1");
  joint_name_.push_back("joint2");
  joint_name_.push_back("joint3");
  joint_name_.push_back("joint4");

  initServiceClient();
  initSubscribe();
}

OpenManipulatorFreedrive::~OpenManipulatorFreedrive()
{
  if (ros::isStarted()) 
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void OpenManipulatorFreedrive::initServiceClient()
{
  set_actuator_state_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
}

bool OpenManipulatorFreedrive::setActuatorState(bool enable)
{
  open_manipulator_msgs::SetActuatorState srv;
//  srv.request.joint_name.push_back(joint_name);
  srv.request.set_actuator_state = enable;

  if (set_actuator_state_client_.call(srv))
  {
    printf("Gelukt\n");
    return srv.response.is_planned;
  }
    printf("Error\n");
  return false;
}



void OpenManipulatorFreedrive::initSubscribe()
{
  open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorFreedrive::jointStatesCallback, this);
}


void OpenManipulatorFreedrive::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for (int i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("gripper"))  temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorFreedrive::setModeState(char ch)
{
  if (ch == '1'){
    mode_state_ = ENABLE_FREEDRIVE;
  }
  else if (ch == '2')
  {
    mode_state_ = DISABLE_FREEDRIVE;
  }
  else if (ch == '3'){
    mode_state_ = EXIT;
  }
}

void OpenManipulatorFreedrive::publishCallback(const ros::TimerEvent&)
{
  printText();
  if (kbhit()) setModeState(std::getchar());

  if (mode_state_ == ENABLE_FREEDRIVE)
  {
      setActuatorState(false);
      mode_state_ = IDLE;
  }
  else if (mode_state_ == DISABLE_FREEDRIVE)
  {
      setActuatorState(true);
      mode_state_ = IDLE;
  }
  else if (mode_state_ == EXIT)
  {
      setActuatorState(true);
      abort_ = true;
      mode_state_ = IDLE;
  }
}


void OpenManipulatorFreedrive::printText()
{
  system("clear");

  printf("\n");
  printf("-----------------------------\n");
  printf("Open manipulator freedive !\n");
  printf("-----------------------------\n");

  printf("Does not work with moveit!\n");

  printf("1 : Enable Freedrive\n");
  printf("2 : Disable Freedrive\n");
  printf("3 : Exit Freedrive\n");

  printf("-----------------------------\n");


  printf("-----------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         present_joint_angle_.at(0),
         present_joint_angle_.at(1),
         present_joint_angle_.at(2),
         present_joint_angle_.at(3));
  printf("Present Tool Position: %.3lf\n", present_joint_angle_.at(4));

}

bool OpenManipulatorFreedrive::abort(){
  return abort_;
}

bool OpenManipulatorFreedrive::kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_freedrive");
  ros::NodeHandle node_handle("");

  OpenManipulatorFreedrive open_manipulator_freedrive;

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/, &OpenManipulatorFreedrive::publishCallback, &open_manipulator_freedrive);

  while (ros::ok())
  {
    ros::spinOnce();
    if(open_manipulator_freedrive.abort()) break;
  }
  return 0;
}
