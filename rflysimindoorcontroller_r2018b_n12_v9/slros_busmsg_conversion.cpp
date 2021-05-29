#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_RflysimIndoorController_r_Point_bd573d and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_RflysimIndoorController_r_Point_bd573d const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Point_bd573d* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_RflysimIndoorController_r_Point32_gvm27s and geometry_msgs::Point32

void convertFromBus(geometry_msgs::Point32* msgPtr, SL_Bus_RflysimIndoorController_r_Point32_gvm27s const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point32");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Point32_gvm27s* busPtr, geometry_msgs::Point32 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point32");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_RflysimIndoorController_r_Pose_y043zc and geometry_msgs::Pose

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_RflysimIndoorController_r_Pose_y043zc const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBus(&msgPtr->position, &busPtr->Position);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Pose_y043zc* busPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBus(&busPtr->Position, &msgPtr->position);
}


// Conversions between SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 and geometry_msgs::PoseStamped

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->pose, &busPtr->Pose);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12* busPtr, geometry_msgs::PoseStamped const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Pose, &msgPtr->pose);
}


// Conversions between SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1 and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_RflysimIndoorController_r_Twist_bfhhhc and geometry_msgs::Twist

void convertFromBus(geometry_msgs::Twist* msgPtr, SL_Bus_RflysimIndoorController_r_Twist_bfhhhc const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertFromBus(&msgPtr->angular, &busPtr->Angular);
  convertFromBus(&msgPtr->linear, &busPtr->Linear);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Twist_bfhhhc* busPtr, geometry_msgs::Twist const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertToBus(&busPtr->Angular, &msgPtr->angular);
  convertToBus(&busPtr->Linear, &msgPtr->linear);
}


// Conversions between SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy and geometry_msgs::TwistStamped

void convertFromBus(geometry_msgs::TwistStamped* msgPtr, SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistStamped");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->twist, &busPtr->Twist);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy* busPtr, geometry_msgs::TwistStamped const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistStamped");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Twist, &msgPtr->twist);
}


// Conversions between SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv and geometry_msgs::Vector3

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv* busPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_RflysimIndoorController_r_Header_94xd8p and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_RflysimIndoorController_r_Header_94xd8p const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Header_94xd8p* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5 and std_msgs::UInt32

void convertFromBus(std_msgs::UInt32* msgPtr, SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5 const* busPtr)
{
  const std::string rosMessageType("std_msgs/UInt32");

  msgPtr->data =  busPtr->Data;
}

void convertToBus(SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5* busPtr, std_msgs::UInt32 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/UInt32");

  busPtr->Data =  msgPtr->data;
}


// Conversions between SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i and std_msgs::UInt64

void convertFromBus(std_msgs::UInt64* msgPtr, SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i const* busPtr)
{
  const std::string rosMessageType("std_msgs/UInt64");

  msgPtr->data = (uint64_t) busPtr->Data;
}

void convertToBus(SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i* busPtr, std_msgs::UInt64 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/UInt64");

  busPtr->Data = (real_T) msgPtr->data;
}


// Conversions between SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 and swarm_msgs::Pipeline

void convertFromBus(swarm_msgs::Pipeline* msgPtr, SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 const* busPtr)
{
  const std::string rosMessageType("swarm_msgs/Pipeline");

  convertFromBus(&msgPtr->pipetype, &busPtr->Pipetype);
  convertFromBusVariableNestedArray(msgPtr->units, busPtr->Units, busPtr->Units_SL_Info);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9* busPtr, swarm_msgs::Pipeline const* msgPtr)
{
  const std::string rosMessageType("swarm_msgs/Pipeline");

  convertToBus(&busPtr->Pipetype, &msgPtr->pipetype);
  convertToBusVariableNestedArray(busPtr->Units, busPtr->Units_SL_Info, msgPtr->units, slros::EnabledWarning(rosMessageType, "units"));
}


// Conversions between SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl and swarm_msgs::Pipeunit

void convertFromBus(swarm_msgs::Pipeunit* msgPtr, SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl const* busPtr)
{
  const std::string rosMessageType("swarm_msgs/Pipeunit");

  convertFromBus(&msgPtr->bottom, &busPtr->Bottom);
  convertFromBus(&msgPtr->left, &busPtr->Left);
  convertFromBus(&msgPtr->middle, &busPtr->Middle);
  convertFromBus(&msgPtr->right, &busPtr->Right);
  convertFromBus(&msgPtr->up, &busPtr->Up);
}

void convertToBus(SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl* busPtr, swarm_msgs::Pipeunit const* msgPtr)
{
  const std::string rosMessageType("swarm_msgs/Pipeunit");

  convertToBus(&busPtr->Bottom, &msgPtr->bottom);
  convertToBus(&busPtr->Left, &msgPtr->left);
  convertToBus(&busPtr->Middle, &msgPtr->middle);
  convertToBus(&busPtr->Right, &msgPtr->right);
  convertToBus(&busPtr->Up, &msgPtr->up);
}

