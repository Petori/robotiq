#ifndef GRIPPERCONTROL_H
#define GRIPPERCONTROL_H

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <ros/ros.h>

ros::Publisher *gripperPubPtr;

void initializeGripperMsg()// Function for gripper controller
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output closeGripperMsg;
    closeGripperMsg.rACT = 0;
    closeGripperMsg.rGTO = 0;
    closeGripperMsg.rATR = 0;
    closeGripperMsg.rPR = 0;
    closeGripperMsg.rSP = 0;
    closeGripperMsg.rFR = 0;
    gripperPubPtr->publish(closeGripperMsg);
    ros::Duration(1).sleep();

    closeGripperMsg.rACT = 1;
    closeGripperMsg.rGTO = 1;
    closeGripperMsg.rSP = 255;
    closeGripperMsg.rFR = 150;
    gripperPubPtr->publish(closeGripperMsg);
    ros::Duration(1).sleep();

}

void sendGripperMsg(int position, int speed=200, int force=150)// Function for gripper controller
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output closeGripperMsg;
    closeGripperMsg.rACT = 1;
    closeGripperMsg.rGTO = 1;
    closeGripperMsg.rATR = 0;
    closeGripperMsg.rPR = std::max(0, std::min(255, position));
    closeGripperMsg.rSP = speed;
    closeGripperMsg.rFR = force;
    gripperPubPtr->publish(closeGripperMsg);
    ros::Duration(1).sleep();
}

#endif // GRIPPERCONTROL_H
