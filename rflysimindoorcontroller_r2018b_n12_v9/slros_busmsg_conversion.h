#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <swarm_msgs/Pipeline.h>
#include <swarm_msgs/Pipeunit.h>
#include "RflysimIndoorController_r2018b_N12_V9_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_RflysimIndoorController_r_Point_bd573d const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Point_bd573d* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Point32* msgPtr, SL_Bus_RflysimIndoorController_r_Point32_gvm27s const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Point32_gvm27s* busPtr, geometry_msgs::Point32 const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_RflysimIndoorController_r_Pose_y043zc const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Pose_y043zc* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12* busPtr, geometry_msgs::PoseStamped const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1 const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(geometry_msgs::Twist* msgPtr, SL_Bus_RflysimIndoorController_r_Twist_bfhhhc const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Twist_bfhhhc* busPtr, geometry_msgs::Twist const* msgPtr);

void convertFromBus(geometry_msgs::TwistStamped* msgPtr, SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy* busPtr, geometry_msgs::TwistStamped const* msgPtr);

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv* busPtr, geometry_msgs::Vector3 const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_RflysimIndoorController_r_Header_94xd8p const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Header_94xd8p* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(std_msgs::UInt32* msgPtr, SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5 const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5* busPtr, std_msgs::UInt32 const* msgPtr);

void convertFromBus(std_msgs::UInt64* msgPtr, SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i* busPtr, std_msgs::UInt64 const* msgPtr);

void convertFromBus(swarm_msgs::Pipeline* msgPtr, SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9* busPtr, swarm_msgs::Pipeline const* msgPtr);

void convertFromBus(swarm_msgs::Pipeunit* msgPtr, SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl const* busPtr);
void convertToBus(SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl* busPtr, swarm_msgs::Pipeunit const* msgPtr);


#endif
