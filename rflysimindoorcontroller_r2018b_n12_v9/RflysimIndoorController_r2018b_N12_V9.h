//
// File: RflysimIndoorController_r2018b_N12_V9.h
//
// Code generated for Simulink model 'RflysimIndoorController_r2018b_N12_V9'.
//
// Model version                  : 1.88
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Dec 25 16:18:22 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_RflysimIndoorController_r2018b_N12_V9_h_
#define RTW_HEADER_RflysimIndoorController_r2018b_N12_V9_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef RflysimIndoorController_r2018b_N12_V9_COMMON_INCLUDES_
# define RflysimIndoorController_r2018b_N12_V9_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // RflysimIndoorController_r2018b_N12_V9_COMMON_INCLUDES_ 

#include "RflysimIndoorController_r2018b_N12_V9_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_defines.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define RflysimIndoorController_r2018b_N12_V9_M (RflysimIndoorController_r201_M)

// Block signals (default storage)
typedef struct {
  real_T p_right_encircle[1000];       // '<Root>/MATLAB Function2'
  real_T p_left_encircle[1000];        // '<Root>/MATLAB Function2'
  real_T p_radius[1000];               // '<Root>/MATLAB Function2'
  real_T nc[1000];
  real_T ki[500];
  real_T dv0[500];
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1;// '<S139>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_a;// '<S138>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_f;// '<S137>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_a4;// '<S136>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_h;// '<S135>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_l;// '<S134>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_g;// '<S133>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_m;// '<S132>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_p;// '<S131>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_n;// '<S130>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_k;// '<S129>/In1'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 In1_d;// '<S128>/In1'
  Bus_type msg_new_m2;                 // '<Root>/MATLAB Function12'
  Bus_type msg_new_pf;                 // '<Root>/MATLAB Function14'
  Bus_type msg_new_m;                  // '<Root>/MATLAB Function13'
  Bus_type msg_new;                    // '<Root>/MATLAB Function9'
  Bus_type msg_new_p;                  // '<Root>/MATLAB Function15'
  Bus_type msg_new_b;                  // '<Root>/MATLAB Function11'
  Bus_type msg_new_i;                  // '<Root>/MATLAB Function5'
  Bus_type msg_new_o;                  // '<Root>/MATLAB Function7'
  Bus_type msg_new_f;                  // '<Root>/MATLAB Function6'
  Bus_type msg_new_mz;                 // '<Root>/MATLAB Function10'
  Bus_type msg_new_j;                  // '<Root>/MATLAB Function8'
  Bus_type msg_new_h;                  // '<Root>/MATLAB Function4'
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 b_varargout_2;
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_b;// '<S103>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_fh;// '<S101>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_lm;// '<S99>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_j;// '<S98>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_hw;// '<S97>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_bp;// '<S89>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_mu;// '<S88>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_nu;// '<S87>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_dh;// '<S86>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_ba;// '<S85>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_fo;// '<S84>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 In1_a4x;// '<S80>/In1'
  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 b_varargout_2_m;
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_dh3;// '<S102>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_m1;// '<S100>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_ai;// '<S96>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_hk;// '<S95>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_mh;// '<S94>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_fz;// '<S93>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_ha;// '<S92>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_jl;// '<S91>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_he;// '<S90>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_e;// '<S83>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_mhe;// '<S82>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy In1_jk;// '<S81>/In1'
  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy BusAssignment1_j;// '<S30>/Bus Assignment1' 
  real_T Vcur[24];                     // '<S16>/MATLAB Function12'
  real_T Pcur[24];                     // '<S16>/MATLAB Function11'
  real_T TmpSignalConversionAtSFun_p[12];// '<Root>/MATLAB Function2'
  real_T TmpSignalConversionAtSFunct[12];// '<Root>/MATLAB Function2'
  real_T uz[12];                       // '<Root>/MATLAB Function2'
  real_T ux[12];                       // '<Root>/MATLAB Function2'
  real_T uy[12];                       // '<Root>/MATLAB Function2'
  real_T uyaw[12];                     // '<Root>/MATLAB Function2'
  real_T states_new[12];               // '<Root>/MATLAB Function2'
  real_T rtb_TmpSignalConversionAtSFun_c[12];
  real_T left_x[10];
  real_T left_y[10];
  real_T right_x[10];
  real_T right_y[10];
  real_T middle_x[10];
  real_T middle_y[10];
  real_T b_k_data[10];
  real_T middle_x_data[10];
  real_T y_data[10];
  real_T x_data[10];
  char_T cv0[47];
  char_T cv1[46];
  char_T cv2[43];
  char_T cv3[42];
  char_T cv4[41];
  char_T cv5[40];
  char_T cv6[24];
  real_T u_encircle[2];
  real_T ep[2];
  real_T V1[2];
  real_T V3[2];
  real_T b_tmp[2];
  real_T Pcur_k[2];
  real_T ux_encircle;
  real_T uy_encircle;
  real_T uyaw_encircle;
  real_T leader_locate;
  real_T dist_encircle;
  real_T dis_leader_min;
  real_T dis_leader;
  real_T rtksii;
  real_T disti;
  real_T dVtidksiti;
  real_T u_tmp;
  real_T u_tmp_c;
  real_T u_tmp_b;
  real_T u_tmp_p;
  real_T u_tmp_tmp;
  real_T u_tmp_tmp_c;
  real_T u_tmp_tmp_f;
  real_T d0;
  real_T d1;
  real_T r;
  real_T d2;
  real_T scale;
  real_T absxk;
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_a1;// '<S127>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_lp;// '<S126>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_b1;// '<S125>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_o;// '<S124>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_hr;// '<S123>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_h3;// '<S122>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_dj;// '<S121>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_ox;// '<S120>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_er;// '<S119>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_fk;// '<S118>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_n0;// '<S117>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i In1_ks;// '<S116>/In1'
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i b_varargout_2_g;
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment;// '<S41>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_g;// '<S40>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_o;// '<S39>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_a;// '<S38>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_i;// '<S37>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_j;// '<S36>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_b;// '<S35>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_e;// '<S34>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_a4;// '<S33>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_et;// '<S32>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_c;// '<S31>/Bus Assignment' 
  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i BusAssignment_k;// '<S30>/Bus Assignment' 
  int32_T b_k_size[2];
  int32_T tri12;                       // '<Root>/MATLAB Function3'
  int32_T tri11;                       // '<Root>/MATLAB Function3'
  int32_T tri10;                       // '<Root>/MATLAB Function3'
  int32_T tri9;                        // '<Root>/MATLAB Function3'
  int32_T tri8;                        // '<Root>/MATLAB Function3'
  int32_T tri7;                        // '<Root>/MATLAB Function3'
  int32_T tri6;                        // '<Root>/MATLAB Function3'
  int32_T tri5;                        // '<Root>/MATLAB Function3'
  uint32_T tunnel_msg_Pipetype_Data;
  uint32_T tunnel_msg_Units_SL_Info_Curren;
  uint32_T k;
} B_RflysimIndoorController_r20_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  Bus_type Memory4_PreviousInput;      // '<Root>/Memory4'
  Bus_type Memory5_PreviousInput;      // '<Root>/Memory5'
  Bus_type Memory6_PreviousInput;      // '<Root>/Memory6'
  Bus_type Memory1_PreviousInput;      // '<Root>/Memory1'
  Bus_type Memory2_PreviousInput;      // '<Root>/Memory2'
  Bus_type Memory3_PreviousInput;      // '<Root>/Memory3'
  Bus_type Memory10_PreviousInput;     // '<Root>/Memory10'
  Bus_type Memory11_PreviousInput;     // '<Root>/Memory11'
  Bus_type Memory12_PreviousInput;     // '<Root>/Memory12'
  Bus_type Memory7_PreviousInput;      // '<Root>/Memory7'
  Bus_type Memory8_PreviousInput;      // '<Root>/Memory8'
  Bus_type Memory9_PreviousInput;      // '<Root>/Memory9'
  robotics_slros_internal_block_T obj; // '<S29>/SourceBlock'
  robotics_slros_internal_block_T obj_p;// '<S28>/SourceBlock'
  robotics_slros_internal_block_T obj_k;// '<S27>/SourceBlock'
  robotics_slros_internal_block_T obj_ps;// '<S26>/SourceBlock'
  robotics_slros_internal_block_T obj_l;// '<S25>/SourceBlock'
  robotics_slros_internal_block_T obj_i;// '<S24>/SourceBlock'
  robotics_slros_internal_block_T obj_b;// '<S23>/SourceBlock'
  robotics_slros_internal_block_T obj_a;// '<S22>/SourceBlock'
  robotics_slros_internal_block_T obj_bb;// '<S21>/SourceBlock'
  robotics_slros_internal_block_T obj_j;// '<S20>/SourceBlock'
  robotics_slros_internal_block_T obj_h;// '<S19>/SourceBlock'
  robotics_slros_internal_block_T obj_aj;// '<S18>/SourceBlock'
  robotics_slros_internal_block_T obj_m;// '<S115>/SourceBlock'
  robotics_slros_internal_block_T obj_n;// '<S114>/SourceBlock'
  robotics_slros_internal_block_T obj_k4;// '<S113>/SourceBlock'
  robotics_slros_internal_block_T obj_k4n;// '<S112>/SourceBlock'
  robotics_slros_internal_block_T obj_f;// '<S111>/SourceBlock'
  robotics_slros_internal_block_T obj_jh;// '<S110>/SourceBlock'
  robotics_slros_internal_block_T obj_mk;// '<S109>/SourceBlock'
  robotics_slros_internal_block_T obj_pf;// '<S108>/SourceBlock'
  robotics_slros_internal_block_T obj_iw;// '<S107>/SourceBlock'
  robotics_slros_internal_block_T obj_o;// '<S106>/SourceBlock'
  robotics_slros_internal_block_T obj_lv;// '<S105>/SourceBlock'
  robotics_slros_internal_block_T obj_n0;// '<S104>/SourceBlock'
  robotics_slros_internal_block_T obj_hv;// '<S79>/SourceBlock'
  robotics_slros_internal_block_T obj_bm;// '<S78>/SourceBlock'
  robotics_slros_internal_block_T obj_nq;// '<S77>/SourceBlock'
  robotics_slros_internal_block_T obj_bbp;// '<S76>/SourceBlock'
  robotics_slros_internal_block_T obj_g;// '<S75>/SourceBlock'
  robotics_slros_internal_block_T obj_ma;// '<S74>/SourceBlock'
  robotics_slros_internal_block_T obj_bs;// '<S73>/SourceBlock'
  robotics_slros_internal_block_T obj_he;// '<S72>/SourceBlock'
  robotics_slros_internal_block_T obj_bp;// '<S71>/SourceBlock'
  robotics_slros_internal_block_T obj_al;// '<S70>/SourceBlock'
  robotics_slros_internal_block_T obj_hc;// '<S69>/SourceBlock'
  robotics_slros_internal_block_T obj_av;// '<S68>/SourceBlock'
  robotics_slros_internal_block_T obj_pa;// '<S67>/SourceBlock'
  robotics_slros_internal_block_T obj_hm;// '<S66>/SourceBlock'
  robotics_slros_internal_block_T obj_d;// '<S65>/SourceBlock'
  robotics_slros_internal_block_T obj_mh;// '<S64>/SourceBlock'
  robotics_slros_internal_block_T obj_e;// '<S63>/SourceBlock'
  robotics_slros_internal_block_T obj_fr;// '<S62>/SourceBlock'
  robotics_slros_internal_block_T obj_jr;// '<S61>/SourceBlock'
  robotics_slros_internal_block_T obj_eu;// '<S60>/SourceBlock'
  robotics_slros_internal_block_T obj_fe;// '<S59>/SourceBlock'
  robotics_slros_internal_block_T obj_c;// '<S58>/SourceBlock'
  robotics_slros_internal_block_T obj_ot;// '<S57>/SourceBlock'
  robotics_slros_internal_block_T obj_dy;// '<S56>/SourceBlock'
  robotics_slros_internal_blo_j_T obj_du;// '<S187>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_by;// '<S186>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_oa;// '<S183>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_a0;// '<S182>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_pd;// '<S179>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_cg;// '<S178>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_gy;// '<S175>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_kr;// '<S174>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_k0;// '<S171>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_k5;// '<S170>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_bc;// '<S167>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_fu;// '<S166>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_kb;// '<S163>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_bt;// '<S162>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_j4;// '<S159>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_cm;// '<S158>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_l0;// '<S155>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_id;// '<S154>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_hx;// '<S151>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_j4t;// '<S150>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_cp;// '<S147>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_hd;// '<S146>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_gn;// '<S143>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_o3;// '<S142>/SinkBlock'
} DW_RflysimIndoorController_r2_T;

// Parameters (default storage)
struct P_RflysimIndoorController_r20_T_ {
  real_T l_v;                          // Variable: l_v
                                       //  Referenced by: '<Root>/Constant7'

  real_T p_target[2];                  // Variable: p_target
                                       //  Referenced by: '<Root>/Constant5'

  real_T ra;                           // Variable: ra
                                       //  Referenced by: '<Root>/Constant10'

  real_T rs;                           // Variable: rs
                                       //  Referenced by: '<Root>/Constant9'

  real_T rt_a;                         // Variable: rt_a
                                       //  Referenced by: '<Root>/Constant2'

  real_T rt_s;                         // Variable: rt_s
                                       //  Referenced by: '<Root>/Constant1'

  real_T v_max;                        // Variable: v_max
                                       //  Referenced by: '<Root>/Constant8'

  Bus_type Memory4_InitialCondition;   // Computed Parameter: Memory4_InitialCondition
                                       //  Referenced by: '<Root>/Memory4'

  Bus_type Memory5_InitialCondition;   // Computed Parameter: Memory5_InitialCondition
                                       //  Referenced by: '<Root>/Memory5'

  Bus_type Memory6_InitialCondition;   // Computed Parameter: Memory6_InitialCondition
                                       //  Referenced by: '<Root>/Memory6'

  Bus_type Memory1_InitialCondition;   // Computed Parameter: Memory1_InitialCondition
                                       //  Referenced by: '<Root>/Memory1'

  Bus_type Memory2_InitialCondition;   // Computed Parameter: Memory2_InitialCondition
                                       //  Referenced by: '<Root>/Memory2'

  Bus_type Memory3_InitialCondition;   // Computed Parameter: Memory3_InitialCondition
                                       //  Referenced by: '<Root>/Memory3'

  Bus_type Memory10_InitialCondition;  // Computed Parameter: Memory10_InitialCondition
                                       //  Referenced by: '<Root>/Memory10'

  Bus_type Memory11_InitialCondition;  // Computed Parameter: Memory11_InitialCondition
                                       //  Referenced by: '<Root>/Memory11'

  Bus_type Memory12_InitialCondition;  // Computed Parameter: Memory12_InitialCondition
                                       //  Referenced by: '<Root>/Memory12'

  Bus_type Memory7_InitialCondition;   // Computed Parameter: Memory7_InitialCondition
                                       //  Referenced by: '<Root>/Memory7'

  Bus_type Memory8_InitialCondition;   // Computed Parameter: Memory8_InitialCondition
                                       //  Referenced by: '<Root>/Memory8'

  Bus_type Memory9_InitialCondition;   // Computed Parameter: Memory9_InitialCondition
                                       //  Referenced by: '<Root>/Memory9'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0;// Computed Parameter: Out1_Y0
                                                           //  Referenced by: '<S128>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value;// Computed Parameter: Constant_Value
                                                                  //  Referenced by: '<S18>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_o;// Computed Parameter: Out1_Y0_o
                                                             //  Referenced by: '<S129>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                    //  Referenced by: '<S19>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_j;// Computed Parameter: Out1_Y0_j
                                                             //  Referenced by: '<S130>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                    //  Referenced by: '<S20>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_h;// Computed Parameter: Out1_Y0_h
                                                             //  Referenced by: '<S131>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                    //  Referenced by: '<S21>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_a;// Computed Parameter: Out1_Y0_a
                                                             //  Referenced by: '<S132>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_k;// Computed Parameter: Constant_Value_k
                                                                    //  Referenced by: '<S22>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_n;// Computed Parameter: Out1_Y0_n
                                                             //  Referenced by: '<S133>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                    //  Referenced by: '<S23>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_jt;// Computed Parameter: Out1_Y0_jt
                                                              //  Referenced by: '<S134>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                                    //  Referenced by: '<S24>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_h3;// Computed Parameter: Out1_Y0_h3
                                                              //  Referenced by: '<S135>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_ok;// Computed Parameter: Constant_Value_ok
                                                                     //  Referenced by: '<S25>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_e;// Computed Parameter: Out1_Y0_e
                                                             //  Referenced by: '<S136>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_g;// Computed Parameter: Constant_Value_g
                                                                    //  Referenced by: '<S26>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_p;// Computed Parameter: Out1_Y0_p
                                                             //  Referenced by: '<S137>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                                    //  Referenced by: '<S27>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_i;// Computed Parameter: Out1_Y0_i
                                                             //  Referenced by: '<S138>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                    //  Referenced by: '<S28>/Constant'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Out1_Y0_op;// Computed Parameter: Out1_Y0_op
                                                              //  Referenced by: '<S139>/Out1'

  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                    //  Referenced by: '<S29>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_eq;// Computed Parameter: Out1_Y0_eq
                                                                 //  Referenced by: '<S80>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S56>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_k;// Computed Parameter: Out1_Y0_k
                                                                //  Referenced by: '<S84>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_ax;// Computed Parameter: Constant_Value_ax
                                                                      //  Referenced by: '<S60>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_c;// Computed Parameter: Out1_Y0_c
                                                                //  Referenced by: '<S85>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_ev;// Computed Parameter: Constant_Value_ev
                                                                      //  Referenced by: '<S61>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_an;// Computed Parameter: Out1_Y0_an
                                                                 //  Referenced by: '<S86>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_aj;// Computed Parameter: Constant_Value_aj
                                                                      //  Referenced by: '<S62>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_l;// Computed Parameter: Out1_Y0_l
                                                                //  Referenced by: '<S87>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_ei;// Computed Parameter: Constant_Value_ei
                                                                      //  Referenced by: '<S63>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_m;// Computed Parameter: Out1_Y0_m
                                                                //  Referenced by: '<S88>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_nf;// Computed Parameter: Constant_Value_nf
                                                                      //  Referenced by: '<S64>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_m1;// Computed Parameter: Out1_Y0_m1
                                                                 //  Referenced by: '<S89>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_k0;// Computed Parameter: Constant_Value_k0
                                                                      //  Referenced by: '<S65>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_ch;// Computed Parameter: Out1_Y0_ch
                                                                 //  Referenced by: '<S97>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_i3;// Computed Parameter: Constant_Value_i3
                                                                      //  Referenced by: '<S73>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_lc;// Computed Parameter: Out1_Y0_lc
                                                                 //  Referenced by: '<S98>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_ix;// Computed Parameter: Constant_Value_ix
                                                                      //  Referenced by: '<S74>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_ni;// Computed Parameter: Out1_Y0_ni
                                                                 //  Referenced by: '<S99>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_iq;// Computed Parameter: Constant_Value_iq
                                                                      //  Referenced by: '<S75>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_e2;// Computed Parameter: Out1_Y0_e2
                                                                 //  Referenced by: '<S101>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_l;// Computed Parameter: Constant_Value_l
                                                                      //  Referenced by: '<S77>/Constant'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Out1_Y0_d;// Computed Parameter: Out1_Y0_d
                                                                //  Referenced by: '<S103>/Out1'

  SL_Bus_RflysimIndoorController_r_PoseStamped_lkfn12 Constant_Value_ao;// Computed Parameter: Constant_Value_ao
                                                                      //  Referenced by: '<S79>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_ng;// Computed Parameter: Out1_Y0_ng
                                                                  //  Referenced by: '<S81>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                      //  Referenced by: '<S57>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_kj;// Computed Parameter: Out1_Y0_kj
                                                                  //  Referenced by: '<S82>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_jt;// Computed Parameter: Constant_Value_jt
                                                                      //  Referenced by: '<S58>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_f;// Computed Parameter: Out1_Y0_f
                                                                 //  Referenced by: '<S83>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_o0;// Computed Parameter: Constant_Value_o0
                                                                      //  Referenced by: '<S59>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_mb;// Computed Parameter: Out1_Y0_mb
                                                                  //  Referenced by: '<S90>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_i2;// Computed Parameter: Constant_Value_i2
                                                                      //  Referenced by: '<S66>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_f4;// Computed Parameter: Out1_Y0_f4
                                                                  //  Referenced by: '<S91>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_c;// Computed Parameter: Constant_Value_c
                                                                      //  Referenced by: '<S67>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_ap;// Computed Parameter: Out1_Y0_ap
                                                                  //  Referenced by: '<S92>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_eo;// Computed Parameter: Constant_Value_eo
                                                                      //  Referenced by: '<S68>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_b;// Computed Parameter: Out1_Y0_b
                                                                 //  Referenced by: '<S93>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_ch;// Computed Parameter: Constant_Value_ch
                                                                      //  Referenced by: '<S69>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_k0;// Computed Parameter: Out1_Y0_k0
                                                                  //  Referenced by: '<S94>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                                      //  Referenced by: '<S70>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_mr;// Computed Parameter: Out1_Y0_mr
                                                                  //  Referenced by: '<S95>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_n0;// Computed Parameter: Constant_Value_n0
                                                                      //  Referenced by: '<S71>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_jz;// Computed Parameter: Out1_Y0_jz
                                                                  //  Referenced by: '<S96>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_lg;// Computed Parameter: Constant_Value_lg
                                                                      //  Referenced by: '<S72>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_fb;// Computed Parameter: Out1_Y0_fb
                                                                  //  Referenced by: '<S100>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_b2;// Computed Parameter: Constant_Value_b2
                                                                      //  Referenced by: '<S76>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Out1_Y0_fm;// Computed Parameter: Out1_Y0_fm
                                                                  //  Referenced by: '<S102>/Out1'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_aor;// Computed Parameter: Constant_Value_aor
                                                                      //  Referenced by: '<S78>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_nc;// Computed Parameter: Constant_Value_nc
                                                                      //  Referenced by: '<S141>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_ak;// Computed Parameter: Constant_Value_ak
                                                                      //  Referenced by: '<S145>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_lo;// Computed Parameter: Constant_Value_lo
                                                                      //  Referenced by: '<S149>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_pp;// Computed Parameter: Constant_Value_pp
                                                                      //  Referenced by: '<S153>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_gd;// Computed Parameter: Constant_Value_gd
                                                                      //  Referenced by: '<S157>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_ej;// Computed Parameter: Constant_Value_ej
                                                                      //  Referenced by: '<S161>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_kd;// Computed Parameter: Constant_Value_kd
                                                                      //  Referenced by: '<S165>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_lj;// Computed Parameter: Constant_Value_lj
                                                                      //  Referenced by: '<S169>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_lp;// Computed Parameter: Constant_Value_lp
                                                                      //  Referenced by: '<S173>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_fx;// Computed Parameter: Constant_Value_fx
                                                                      //  Referenced by: '<S177>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_f3;// Computed Parameter: Constant_Value_f3
                                                                      //  Referenced by: '<S181>/Constant'

  SL_Bus_RflysimIndoorController_r_TwistStamped_qujgvy Constant_Value_be;// Computed Parameter: Constant_Value_be
                                                                      //  Referenced by: '<S185>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_c5;// Computed Parameter: Out1_Y0_c5
                                                            //  Referenced by: '<S116>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_p0;// Computed Parameter: Constant_Value_p0
                                                                   //  Referenced by: '<S104>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_bn;// Computed Parameter: Out1_Y0_bn
                                                            //  Referenced by: '<S117>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_m;// Computed Parameter: Constant_Value_m
                                                                  //  Referenced by: '<S105>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_kn;// Computed Parameter: Out1_Y0_kn
                                                            //  Referenced by: '<S118>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_jr;// Computed Parameter: Constant_Value_jr
                                                                   //  Referenced by: '<S106>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_ek;// Computed Parameter: Out1_Y0_ek
                                                            //  Referenced by: '<S119>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_h1;// Computed Parameter: Constant_Value_h1
                                                                   //  Referenced by: '<S107>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_kt;// Computed Parameter: Out1_Y0_kt
                                                            //  Referenced by: '<S120>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_p4;// Computed Parameter: Constant_Value_p4
                                                                   //  Referenced by: '<S108>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_cr;// Computed Parameter: Out1_Y0_cr
                                                            //  Referenced by: '<S121>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_kr;// Computed Parameter: Constant_Value_kr
                                                                   //  Referenced by: '<S109>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_hx;// Computed Parameter: Out1_Y0_hx
                                                            //  Referenced by: '<S122>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_cl;// Computed Parameter: Constant_Value_cl
                                                                   //  Referenced by: '<S110>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_os;// Computed Parameter: Out1_Y0_os
                                                            //  Referenced by: '<S123>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_n1;// Computed Parameter: Constant_Value_n1
                                                                   //  Referenced by: '<S111>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_jz5;// Computed Parameter: Out1_Y0_jz5
                                                             //  Referenced by: '<S124>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_gz;// Computed Parameter: Constant_Value_gz
                                                                   //  Referenced by: '<S112>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_hm;// Computed Parameter: Out1_Y0_hm
                                                            //  Referenced by: '<S125>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_gr;// Computed Parameter: Constant_Value_gr
                                                                   //  Referenced by: '<S113>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_ac;// Computed Parameter: Out1_Y0_ac
                                                            //  Referenced by: '<S126>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_j5;// Computed Parameter: Constant_Value_j5
                                                                   //  Referenced by: '<S114>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Out1_Y0_lz;// Computed Parameter: Out1_Y0_lz
                                                            //  Referenced by: '<S127>/Out1'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_dk;// Computed Parameter: Constant_Value_dk
                                                                   //  Referenced by: '<S115>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_pm;// Computed Parameter: Constant_Value_pm
                                                                   //  Referenced by: '<S140>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_ik;// Computed Parameter: Constant_Value_ik
                                                                   //  Referenced by: '<S144>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_lf;// Computed Parameter: Constant_Value_lf
                                                                   //  Referenced by: '<S148>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_cm;// Computed Parameter: Constant_Value_cm
                                                                   //  Referenced by: '<S152>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_md;// Computed Parameter: Constant_Value_md
                                                                   //  Referenced by: '<S156>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_ep;// Computed Parameter: Constant_Value_ep
                                                                   //  Referenced by: '<S160>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_jj;// Computed Parameter: Constant_Value_jj
                                                                   //  Referenced by: '<S164>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_lfo;// Computed Parameter: Constant_Value_lfo
                                                                    //  Referenced by: '<S168>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_mo;// Computed Parameter: Constant_Value_mo
                                                                   //  Referenced by: '<S172>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_hg;// Computed Parameter: Constant_Value_hg
                                                                   //  Referenced by: '<S176>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_h2;// Computed Parameter: Constant_Value_h2
                                                                   //  Referenced by: '<S180>/Constant'

  SL_Bus_RflysimIndoorController_r_UInt64_3ehx7i Constant_Value_km;// Computed Parameter: Constant_Value_km
                                                                   //  Referenced by: '<S184>/Constant'

  real_T SimulationPace_P1;            // Expression: SimulationPace
                                       //  Referenced by: '<Root>/Simulation Pace'

  real_T SimulationPace_P2;            // Expression: SleepMode
                                       //  Referenced by: '<Root>/Simulation Pace'

  real_T SimulationPace_P3;            // Expression: OutputPaceError
                                       //  Referenced by: '<Root>/Simulation Pace'

  real_T SimulationPace_P4;            // Expression: SampleTime
                                       //  Referenced by: '<Root>/Simulation Pace'

};

// Real-time Model Data Structure
struct tag_RTM_RflysimIndoorControll_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_RflysimIndoorController_r20_T RflysimIndoorController_r2018_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
extern B_RflysimIndoorController_r20_T RflysimIndoorController_r2018_B;

// Block states (default storage)
extern DW_RflysimIndoorController_r2_T RflysimIndoorController_r201_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void RflysimIndoorController_r2018b_N12_V9_initialize(void);
  extern void RflysimIndoorController_r2018b_N12_V9_step(void);
  extern void RflysimIndoorController_r2018b_N12_V9_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_RflysimIndoorControl_T *const RflysimIndoorController_r201_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Display6' : Unused code path elimination
//  Block '<Root>/Display8' : Unused code path elimination
//  Block '<S16>/Display1' : Unused code path elimination
//  Block '<S16>/Display10' : Unused code path elimination
//  Block '<S16>/Display11' : Unused code path elimination
//  Block '<S16>/Display12' : Unused code path elimination
//  Block '<S16>/Display2' : Unused code path elimination
//  Block '<S16>/Display3' : Unused code path elimination
//  Block '<S16>/Display4' : Unused code path elimination
//  Block '<S16>/Display5' : Unused code path elimination
//  Block '<S16>/Display6' : Unused code path elimination
//  Block '<S16>/Display7' : Unused code path elimination
//  Block '<S16>/Display8' : Unused code path elimination
//  Block '<S16>/Display9' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'RflysimIndoorController_r2018b_N12_V9'
//  '<S1>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function1'
//  '<S2>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function10'
//  '<S3>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function11'
//  '<S4>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function12'
//  '<S5>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function13'
//  '<S6>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function14'
//  '<S7>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function15'
//  '<S8>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function2'
//  '<S9>'   : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function3'
//  '<S10>'  : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function4'
//  '<S11>'  : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function5'
//  '<S12>'  : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function6'
//  '<S13>'  : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function7'
//  '<S14>'  : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function8'
//  '<S15>'  : 'RflysimIndoorController_r2018b_N12_V9/MATLAB Function9'
//  '<S16>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity'
//  '<S17>'  : 'RflysimIndoorController_r2018b_N12_V9/States'
//  '<S18>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe1'
//  '<S19>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe10'
//  '<S20>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe11'
//  '<S21>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe12'
//  '<S22>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe2'
//  '<S23>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe3'
//  '<S24>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe4'
//  '<S25>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe5'
//  '<S26>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe6'
//  '<S27>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe7'
//  '<S28>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe8'
//  '<S29>'  : 'RflysimIndoorController_r2018b_N12_V9/Subscribe9'
//  '<S30>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem'
//  '<S31>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem1'
//  '<S32>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem10'
//  '<S33>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem11'
//  '<S34>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem2'
//  '<S35>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem3'
//  '<S36>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem4'
//  '<S37>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem5'
//  '<S38>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem6'
//  '<S39>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem7'
//  '<S40>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem8'
//  '<S41>'  : 'RflysimIndoorController_r2018b_N12_V9/Subsystem9'
//  '<S42>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function'
//  '<S43>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function1'
//  '<S44>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function10'
//  '<S45>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function11'
//  '<S46>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function12'
//  '<S47>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function13'
//  '<S48>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function2'
//  '<S49>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function3'
//  '<S50>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function4'
//  '<S51>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function5'
//  '<S52>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function6'
//  '<S53>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function7'
//  '<S54>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function8'
//  '<S55>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/MATLAB Function9'
//  '<S56>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe1'
//  '<S57>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe10'
//  '<S58>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe11'
//  '<S59>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe12'
//  '<S60>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe13'
//  '<S61>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe14'
//  '<S62>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe15'
//  '<S63>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe16'
//  '<S64>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe17'
//  '<S65>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe18'
//  '<S66>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe19'
//  '<S67>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe2'
//  '<S68>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe20'
//  '<S69>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe21'
//  '<S70>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe22'
//  '<S71>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe23'
//  '<S72>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe24'
//  '<S73>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe3'
//  '<S74>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe4'
//  '<S75>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe5'
//  '<S76>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe6'
//  '<S77>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe7'
//  '<S78>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe8'
//  '<S79>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe9'
//  '<S80>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe1/Enabled Subsystem'
//  '<S81>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe10/Enabled Subsystem'
//  '<S82>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe11/Enabled Subsystem'
//  '<S83>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe12/Enabled Subsystem'
//  '<S84>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe13/Enabled Subsystem'
//  '<S85>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe14/Enabled Subsystem'
//  '<S86>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe15/Enabled Subsystem'
//  '<S87>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe16/Enabled Subsystem'
//  '<S88>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe17/Enabled Subsystem'
//  '<S89>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe18/Enabled Subsystem'
//  '<S90>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe19/Enabled Subsystem'
//  '<S91>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe2/Enabled Subsystem'
//  '<S92>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe20/Enabled Subsystem'
//  '<S93>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe21/Enabled Subsystem'
//  '<S94>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe22/Enabled Subsystem'
//  '<S95>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe23/Enabled Subsystem'
//  '<S96>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe24/Enabled Subsystem'
//  '<S97>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe3/Enabled Subsystem'
//  '<S98>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe4/Enabled Subsystem'
//  '<S99>'  : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe5/Enabled Subsystem'
//  '<S100>' : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe6/Enabled Subsystem'
//  '<S101>' : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe7/Enabled Subsystem'
//  '<S102>' : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe8/Enabled Subsystem'
//  '<S103>' : 'RflysimIndoorController_r2018b_N12_V9/Position, Yaw and Velocity/Subscribe9/Enabled Subsystem'
//  '<S104>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe'
//  '<S105>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe1'
//  '<S106>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe10'
//  '<S107>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe11'
//  '<S108>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe2'
//  '<S109>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe3'
//  '<S110>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe4'
//  '<S111>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe5'
//  '<S112>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe6'
//  '<S113>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe7'
//  '<S114>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe8'
//  '<S115>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe9'
//  '<S116>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe/Enabled Subsystem'
//  '<S117>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe1/Enabled Subsystem'
//  '<S118>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe10/Enabled Subsystem'
//  '<S119>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe11/Enabled Subsystem'
//  '<S120>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe2/Enabled Subsystem'
//  '<S121>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe3/Enabled Subsystem'
//  '<S122>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe4/Enabled Subsystem'
//  '<S123>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe5/Enabled Subsystem'
//  '<S124>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe6/Enabled Subsystem'
//  '<S125>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe7/Enabled Subsystem'
//  '<S126>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe8/Enabled Subsystem'
//  '<S127>' : 'RflysimIndoorController_r2018b_N12_V9/States/Subscribe9/Enabled Subsystem'
//  '<S128>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe1/Enabled Subsystem'
//  '<S129>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe10/Enabled Subsystem'
//  '<S130>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe11/Enabled Subsystem'
//  '<S131>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe12/Enabled Subsystem'
//  '<S132>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe2/Enabled Subsystem'
//  '<S133>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe3/Enabled Subsystem'
//  '<S134>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe4/Enabled Subsystem'
//  '<S135>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe5/Enabled Subsystem'
//  '<S136>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe6/Enabled Subsystem'
//  '<S137>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe7/Enabled Subsystem'
//  '<S138>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe8/Enabled Subsystem'
//  '<S139>' : 'RflysimIndoorController_r2018b_N12_V9/Subscribe9/Enabled Subsystem'
//  '<S140>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem/Blank Message'
//  '<S141>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem/Blank Message1'
//  '<S142>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem/Publish'
//  '<S143>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem/Publish1'
//  '<S144>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem1/Blank Message'
//  '<S145>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem1/Blank Message1'
//  '<S146>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem1/Publish'
//  '<S147>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem1/Publish1'
//  '<S148>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem10/Blank Message'
//  '<S149>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem10/Blank Message1'
//  '<S150>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem10/Publish'
//  '<S151>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem10/Publish1'
//  '<S152>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem11/Blank Message'
//  '<S153>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem11/Blank Message1'
//  '<S154>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem11/Publish'
//  '<S155>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem11/Publish1'
//  '<S156>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem2/Blank Message'
//  '<S157>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem2/Blank Message1'
//  '<S158>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem2/Publish'
//  '<S159>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem2/Publish1'
//  '<S160>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem3/Blank Message'
//  '<S161>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem3/Blank Message1'
//  '<S162>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem3/Publish'
//  '<S163>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem3/Publish1'
//  '<S164>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem4/Blank Message'
//  '<S165>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem4/Blank Message1'
//  '<S166>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem4/Publish'
//  '<S167>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem4/Publish1'
//  '<S168>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem5/Blank Message'
//  '<S169>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem5/Blank Message1'
//  '<S170>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem5/Publish'
//  '<S171>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem5/Publish1'
//  '<S172>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem6/Blank Message'
//  '<S173>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem6/Blank Message1'
//  '<S174>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem6/Publish'
//  '<S175>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem6/Publish1'
//  '<S176>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem7/Blank Message'
//  '<S177>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem7/Blank Message1'
//  '<S178>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem7/Publish'
//  '<S179>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem7/Publish1'
//  '<S180>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem8/Blank Message'
//  '<S181>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem8/Blank Message1'
//  '<S182>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem8/Publish'
//  '<S183>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem8/Publish1'
//  '<S184>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem9/Blank Message'
//  '<S185>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem9/Blank Message1'
//  '<S186>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem9/Publish'
//  '<S187>' : 'RflysimIndoorController_r2018b_N12_V9/Subsystem9/Publish1'

#endif                                 // RTW_HEADER_RflysimIndoorController_r2018b_N12_V9_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
