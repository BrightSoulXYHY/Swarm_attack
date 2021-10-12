//
// File: RflysimIndoorController_r2018b_N12_V9_types.h
//
// Code generated for Simulink model 'RflysimIndoorController_r2018b_N12_V9'.
//
// Model version                  : 1.87
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Oct 12 16:43:55 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_RflysimIndoorController_r2018b_N12_V9_types_h_
#define RTW_HEADER_RflysimIndoorController_r2018b_N12_V9_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5_

// MsgType=std_msgs/UInt32
typedef struct {
  uint32_T Data;
} SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Point32_gvm27s_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Point32_gvm27s_

// MsgType=geometry_msgs/Point32
typedef struct {
  real32_T X;
  real32_T Y;
  real32_T Z;
} SL_Bus_RflysimIndoorController_r_Point32_gvm27s;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl_

// MsgType=swarm_msgs/Pipeunit
typedef struct {
  // MsgType=geometry_msgs/Point32
  SL_Bus_RflysimIndoorController_r_Point32_gvm27s Middle;

  // MsgType=geometry_msgs/Point32
  SL_Bus_RflysimIndoorController_r_Point32_gvm27s Left;

  // MsgType=geometry_msgs/Point32
  SL_Bus_RflysimIndoorController_r_Point32_gvm27s Right;

  // MsgType=geometry_msgs/Point32
  SL_Bus_RflysimIndoorController_r_Point32_gvm27s Bottom;

  // MsgType=geometry_msgs/Point32
  SL_Bus_RflysimIndoorController_r_Point32_gvm27s Up;
} SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Bus_type_
#define DEFINED_TYPEDEF_FOR_Bus_type_

// MsgType=swarm_msgs/Pipeline
typedef struct {
  // MsgType=std_msgs/UInt32
  SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5 Pipetype;

  // MsgType=swarm_msgs/Pipeunit:IsVarLen=1:VarLenCategory=data:VarLenElem=Units_SL_Info:TruncateAction=warn 
  SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl Units[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Units
  SL_Bus_ROSVariableLengthArrayInfo Units_SL_Info;
} Bus_type;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9_

// MsgType=swarm_msgs/Pipeline
typedef struct {
  // MsgType=std_msgs/UInt32
  SL_Bus_RflysimIndoorController_r_UInt32_3ehxa5 Pipetype;

  // MsgType=swarm_msgs/Pipeunit:IsVarLen=1:VarLenCategory=data:VarLenElem=Units_SL_Info:TruncateAction=warn 
  SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl Units[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Units
  SL_Bus_ROSVariableLengthArrayInfo Units_SL_Info;
} SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Header_94xd8p_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Header_94xd8p_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_RflysimIndoorController_r2018b_N12_V9_ros_time_Time Stamp;
} SL_Bus_RflysimIndoorController_r_Header_94xd8p;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Point_bd573d_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Point_bd573d_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_RflysimIndoorController_r_Point_bd573d;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1_

// MsgType=geometry_msgs/Quaternion
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Pose_y043zc_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Pose_y043zc_

// MsgType=geometry_msgs/Pose
typedef struct {
  // MsgType=geometry_msgs/Point
  SL_Bus_RflysimIndoorController_r_Point_bd573d Position;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1 Orientation;
} SL_Bus_RflysimIndoorController_r_Pose_y043zc;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12_

// MsgType=geometry_msgs/PoseStamped
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_RflysimIndoorController_r_Header_94xd8p Header;

  // MsgType=geometry_msgs/Pose
  SL_Bus_RflysimIndoorController_r_Pose_y043zc Pose;
} SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Twist_bfhhhc_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_Twist_bfhhhc_

// MsgType=geometry_msgs/Twist
typedef struct {
  // MsgType=geometry_msgs/Vector3
  SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv Linear;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_RflysimIndoorController_r_Vector3_4nzmmv Angular;
} SL_Bus_RflysimIndoorController_r_Twist_bfhhhc;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy_

// MsgType=geometry_msgs/TwistStamped
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_RflysimIndoorController_r_Header_94xd8p Header;

  // MsgType=geometry_msgs/Twist
  SL_Bus_RflysimIndoorController_r_Twist_bfhhhc Twist;
} SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i_
#define DEFINED_TYPEDEF_FOR_SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i_

// MsgType=std_msgs/UInt64
typedef struct {
  // Int64Type=uint64
  real_T Data;
} SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_j_T
#define typedef_robotics_slros_internal_blo_j_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_j_T;

#endif                                 //typedef_robotics_slros_internal_blo_j_T

// Parameters (default storage)
typedef struct P_RflysimIndoorController_r20_T_ P_RflysimIndoorController_r20_T;

// Forward declaration for rtModel
typedef struct tag_RTM_RflysimIndoorControll_T RT_MODEL_RflysimIndoorControl_T;

#endif                                 // RTW_HEADER_RflysimIndoorController_r2018b_N12_V9_types_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
