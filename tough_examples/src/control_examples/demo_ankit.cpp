#include <tough_common/robot_description.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_footstep/robot_walker.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>

void setArmPose(geometry_msgs::Pose& arm_pose)
{
  arm_pose.orientation.w = 1;
  arm_pose.position.x = 0.5;
  arm_pose.position.y = -0.5;
  arm_pose.position.z = 0.2;
}

std::string showPrompt(int stage)
{
  std::string prompt = "";
  switch (stage)
  {
    case 0:
      prompt = "Exit code.";
      break;

    case 1:
      prompt = "Walk robot by 4 steps and move hand";
      break;

    case 2:
      prompt = "Move Hand to 0.5 -0.5 0.2";
      break;

    case 3:
      prompt = "Bend chest with yaw and pitch of 10`";
      break;

    case 4:
      prompt = "Reset robot pose";
      break;

    case 5:
      prompt = "Lift Right leg";
      break;

    case 6:
      prompt = "Move Hand to 0.5 -0.5 0.2";
      break;

    case 7:
      prompt = "Bend chest with yaw and pitch of 10`";
      break;

    case 8:
      prompt = "Reset Pose";
      break;

    case 9:
      prompt = "Place Leg";
      break;

    case 10:
      prompt = "Restart Task";
      break;

    default:
      break;
  }
  return prompt;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "atlas_demo_node");
  ros::NodeHandle nh;

  int stage = 1;
  char action;
  bool execute_code = true;
  const double TO_RADIANS = M_PI / 180.0;
  float ten_degrees_in_radians = 10 * TO_RADIANS;
  geometry_msgs::Pose arm_pose;

  ArmControlInterface arm_controller(nh);
  ArmControlInterface armTraj(nh);
  ChestControlInterface chest_controller(nh);
  GripperControlInterface gripper_controller(nh);
  RobotWalker robot_walker(nh, 0.8, 0.8, 0, 0.18);

  setArmPose(arm_pose);

  while (execute_code)
  {
    std::string curr_prompt = showPrompt(stage), prev_prompt = showPrompt(stage - 1);
    std::cout << std::endl << std::endl << "Please enter" << std::endl;
    std::cout << "'n' or Enter to :   " << curr_prompt << std::endl;
    std::cout << "'p' to          :   " << prev_prompt << std::endl;
    std::cout << "'q' or Esc to   :   Exit Code" << std::endl;
    action = std::cin.get();

    if (action != '\n')
      std::cin.ignore();

    if (action == 'p')
      stage--;
    else if (action == 27 || action == 'q')  // ESC is 27
    {
      std::cout << "Exiting the code" << std::endl;
      break;
    }
    else if ((action != 'n') && (action != '\n'))
    {
      std::cout << "Invalid input. Input again" << std::endl;
      // std::cin.ignore();
      continue;
    }

    switch (stage)
    {
      case 1:
        std::cout << "Walking 4 steps and moving hand" << std::endl;
        robot_walker.walkNStepsWRTPelvis(4, 0.3);
        std::cout << "Moving Hand" << std::endl;
        armTraj.moveArmJoint(LEFT, 4, 0.5);
        ros::Duration(3.0f).sleep();
        gripper_controller.openGripper(RobotSide::RIGHT);
        ros::Duration(1.0f).sleep();
        break;

      case 2:
        std::cout << "Moving Hand" << std::endl;
        arm_controller.moveArmInTaskSpace(RobotSide::RIGHT, arm_pose, 3.0);
        ros::Duration(3.0f).sleep();
        gripper_controller.openGripper(RobotSide::RIGHT);
        ros::Duration(1.0f).sleep();
        break;

      case 3:
        std::cout << "Moving Chest" << std::endl;
        chest_controller.controlChest(0, ten_degrees_in_radians, ten_degrees_in_radians, 1.5);
        ros::Duration(1.5f).sleep();
        break;

      case 4:
        std::cout << "resetting pose" << std::endl;
        chest_controller.resetPose(2.0);
        ros::Duration(2.0f).sleep();
        gripper_controller.closeGripper(RobotSide::RIGHT);
        arm_controller.moveToDefaultPose(RobotSide::RIGHT, 2.0);
        ros::Duration(2.0f).sleep();
        break;

      case 5:
        std::cout << "Lifting Right Leg" << std::endl;
        robot_walker.raiseLeg(RobotSide::RIGHT, 0.2, 2.0);
        ros::Duration(2.0f).sleep();
        break;

      case 6:
        std::cout << "Moving Hand" << std::endl;
        arm_controller.moveArmInTaskSpace(RobotSide::RIGHT, arm_pose, 3.0);
        ros::Duration(3.0).sleep();
        gripper_controller.openGripper(RobotSide::RIGHT);
        ros::Duration(1.0).sleep();
        break;

      case 7:
        std::cout << "Moving Chest" << std::endl;
        chest_controller.controlChest(0, ten_degrees_in_radians, ten_degrees_in_radians, 1.0);
        ros::Duration(1.0f).sleep();
        break;

      case 8:
        std::cout << "Reseting Pose" << std::endl;
        chest_controller.resetPose(2.0);
        ros::Duration(2.0f).sleep();
        gripper_controller.closeGripper(RobotSide::RIGHT);
        arm_controller.moveToDefaultPose(RobotSide::RIGHT, 2.0);
        ros::Duration(2.0f).sleep();
        break;

      case 9:
        std::cout << "Placing leg " << std::endl;
        robot_walker.placeLeg(RobotSide::RIGHT, 0.2, 2.0);
        ros::Duration(3.0f).sleep();
        break;

      case 10:
        stage = 0;
        break;

      default:
        std::cout << "Exiting the code" << std::endl;
        execute_code = false;
        break;
    }
    stage++;
  }

  return 0;
}