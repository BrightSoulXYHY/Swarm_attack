//
// File: RflysimIndoorController_r2018b_N12_V9.cpp
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
#include "RflysimIndoorController_r2018b_N12_V9.h"
#include "RflysimIndoorController_r2018b_N12_V9_private.h"

// Block signals (default storage)
B_RflysimIndoorController_r20_T RflysimIndoorController_r2018_B;

// Block states (default storage)
DW_RflysimIndoorController_r2_T RflysimIndoorController_r201_DW;

// Real-time model
RT_MODEL_RflysimIndoorControl_T RflysimIndoorController_r201_M_;
RT_MODEL_RflysimIndoorControl_T *const RflysimIndoorController_r201_M =
  &RflysimIndoorController_r201_M_;

// Forward declaration for local functions
static real_T RflysimIndoorControlle_mysatNum(real_T x);
static void RflysimIndoorControlle_linspace(real_T d2, real_T y[500]);
static void RflysimIndoorController_interp1(const real_T varargin_1_data[],
  const int32_T varargin_1_size[2], const real_T varargin_2_data[], const
  int32_T *varargin_2_size, const real_T varargin_3[500], real_T Vq[500]);
static real_T RflysimIndoorContr_minAngleDiff(real_T a, real_T b);
static real_T RflysimIndoorController_r2_norm(const real_T x[2]);
static real_T RflysimIndoorContro_rt_powd_snf(real_T u0, real_T u1);
static real_T RflysimIndoorControll_dmysigma2(real_T x, real_T d1, real_T d2);
static real_T RflysimIndoorController_r20_dot(const real_T a[2], const real_T b
  [2]);
static void RflysimIndoorCont_Encirclement4(const real_T p_leader[1000], const
  real_T tunnel_left[1000], const real_T tunnel_right[1000], const real_T
  p_target[2], const real_T Pcur[24], const real_T Vcur[24], const real_T
  Yawcur[12], real_T gain, real_T vmax, real_T rs, real_T ra, real_T rt1, real_T
  rt2, real_T i, real_T u[2], real_T *ux, real_T *uy, real_T *uyaw, real_T
  *leader_locate, real_T *dist);
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);
static void RflysimIndoo_matlabCodegenHa_i1(robotics_slros_internal_blo_j_T *obj);

//
// Output and update for atomic system:
//    '<Root>/MATLAB Function10'
//    '<Root>/MATLAB Function11'
//    '<Root>/MATLAB Function12'
//    '<Root>/MATLAB Function13'
//    '<Root>/MATLAB Function14'
//    '<Root>/MATLAB Function15'
//    '<Root>/MATLAB Function4'
//    '<Root>/MATLAB Function5'
//    '<Root>/MATLAB Function6'
//    '<Root>/MATLAB Function7'
//    ...
//
void RflysimIndoorC_MATLABFunction10(const
  SL_Bus_RflysimIndoorController_r_Pipeline_6g4ty9 *rtu_msg, const Bus_type
  *rtu_msg_last, Bus_type *rty_msg_new)
{
  if (rtu_msg->Units_SL_Info.CurrentLength == 0U) {
    *rty_msg_new = *rtu_msg_last;
  } else {
    rty_msg_new->Pipetype = rtu_msg->Pipetype;
    memcpy(&rty_msg_new->Units[0], &rtu_msg->Units[0], sizeof
           (SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl) << 4U);
    rty_msg_new->Units_SL_Info = rtu_msg->Units_SL_Info;
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Output and update for atomic system:
//    '<S16>/MATLAB Function'
//    '<S16>/MATLAB Function1'
//    '<S16>/MATLAB Function10'
//    '<S16>/MATLAB Function13'
//    '<S16>/MATLAB Function2'
//    '<S16>/MATLAB Function3'
//    '<S16>/MATLAB Function4'
//    '<S16>/MATLAB Function5'
//    '<S16>/MATLAB Function6'
//    '<S16>/MATLAB Function7'
//    ...
//
void RflysimIndoorCon_MATLABFunction(const
  SL_Bus_RflysimIndoorController_r_Quaternion_z85xd1 *rtu_u, real_T *rty_yaw)
{
  *rty_yaw = rt_atan2d_snf((rtu_u->W * rtu_u->Z + rtu_u->X * rtu_u->Y) * 2.0,
    1.0 - (rtu_u->Y * rtu_u->Y + rtu_u->Z * rtu_u->Z) * 2.0);
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static real_T RflysimIndoorControlle_mysatNum(real_T x)
{
  real_T u;
  if (x > 2.0) {
    u = 2.0;
  } else if (x < -2.0) {
    u = -2.0;
  } else {
    u = x;
  }

  return u;
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static void RflysimIndoorControlle_linspace(real_T d2, real_T y[500])
{
  real_T delta1;
  int32_T k;
  y[499] = d2;
  y[0] = 1.0;
  delta1 = (d2 - 1.0) / 499.0;
  for (k = 0; k < 498; k++) {
    y[k + 1] = (1.0 + (real_T)k) * delta1 + 1.0;
  }
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static void RflysimIndoorController_interp1(const real_T varargin_1_data[],
  const int32_T varargin_1_size[2], const real_T varargin_2_data[], const
  int32_T *varargin_2_size, const real_T varargin_3[500], real_T Vq[500])
{
  int32_T nx;
  int32_T nd2;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  int32_T x_size_idx_1;
  int32_T exitg1;
  if (0 <= *varargin_2_size - 1) {
    memcpy(&RflysimIndoorController_r2018_B.y_data[0], &varargin_2_data[0],
           *varargin_2_size * sizeof(real_T));
  }

  x_size_idx_1 = varargin_1_size[1];
  nx = varargin_1_size[0] * varargin_1_size[1] - 1;
  if (0 <= nx) {
    memcpy(&RflysimIndoorController_r2018_B.x_data[0], &varargin_1_data[0], (nx
            + 1) * sizeof(real_T));
  }

  nx = varargin_1_size[1] - 1;
  nd2 = 0;
  do {
    exitg1 = 0;
    if (nd2 <= nx) {
      if (rtIsNaN(varargin_1_data[nd2])) {
        exitg1 = 1;
      } else {
        nd2++;
      }
    } else {
      if (varargin_1_data[1] < varargin_1_data[0]) {
        nd2 = (nx + 1) >> 1;
        for (low_ip1 = 0; low_ip1 < nd2; low_ip1++) {
          RflysimIndoorController_r2018_B.r =
            RflysimIndoorController_r2018_B.x_data[low_ip1];
          high_i = nx - low_ip1;
          RflysimIndoorController_r2018_B.x_data[low_ip1] =
            RflysimIndoorController_r2018_B.x_data[high_i];
          RflysimIndoorController_r2018_B.x_data[high_i] =
            RflysimIndoorController_r2018_B.r;
        }

        if ((*varargin_2_size != 0) && (*varargin_2_size > 1)) {
          nx = *varargin_2_size - 1;
          nd2 = *varargin_2_size >> 1;
          for (low_ip1 = 0; low_ip1 < nd2; low_ip1++) {
            RflysimIndoorController_r2018_B.r =
              RflysimIndoorController_r2018_B.y_data[low_ip1];
            high_i = nx - low_ip1;
            RflysimIndoorController_r2018_B.y_data[low_ip1] =
              RflysimIndoorController_r2018_B.y_data[high_i];
            RflysimIndoorController_r2018_B.y_data[high_i] =
              RflysimIndoorController_r2018_B.r;
          }
        }
      }

      for (nx = 0; nx < 500; nx++) {
        Vq[nx] = (rtNaN);
        if ((!(varargin_3[nx] >
               RflysimIndoorController_r2018_B.x_data[x_size_idx_1 - 1])) &&
            (!(varargin_3[nx] < RflysimIndoorController_r2018_B.x_data[0]))) {
          nd2 = 1;
          low_ip1 = 2;
          high_i = x_size_idx_1;
          while (high_i > low_ip1) {
            mid_i = (nd2 >> 1) + (high_i >> 1);
            if (((nd2 & 1) == 1) && ((high_i & 1) == 1)) {
              mid_i++;
            }

            if (varargin_3[nx] >= RflysimIndoorController_r2018_B.x_data[mid_i -
                1]) {
              nd2 = mid_i;
              low_ip1 = mid_i + 1;
            } else {
              high_i = mid_i;
            }
          }

          RflysimIndoorController_r2018_B.r =
            RflysimIndoorController_r2018_B.x_data[nd2 - 1];
          RflysimIndoorController_r2018_B.r = (varargin_3[nx] -
            RflysimIndoorController_r2018_B.r) /
            (RflysimIndoorController_r2018_B.x_data[nd2] -
             RflysimIndoorController_r2018_B.r);
          if (RflysimIndoorController_r2018_B.r == 0.0) {
            Vq[nx] = RflysimIndoorController_r2018_B.y_data[nd2 - 1];
          } else if (RflysimIndoorController_r2018_B.r == 1.0) {
            Vq[nx] = RflysimIndoorController_r2018_B.y_data[nd2];
          } else {
            RflysimIndoorController_r2018_B.d2 =
              RflysimIndoorController_r2018_B.y_data[nd2 - 1];
            if (RflysimIndoorController_r2018_B.d2 ==
                RflysimIndoorController_r2018_B.y_data[nd2]) {
              Vq[nx] = RflysimIndoorController_r2018_B.d2;
            } else {
              Vq[nx] = (1.0 - RflysimIndoorController_r2018_B.r) *
                RflysimIndoorController_r2018_B.d2 +
                RflysimIndoorController_r2018_B.r *
                RflysimIndoorController_r2018_B.y_data[nd2];
            }
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static real_T RflysimIndoorContr_minAngleDiff(real_T a, real_T b)
{
  real_T diffab;
  diffab = a - b;
  if (diffab < 0.0) {
    diffab += 6.2831853071795862;
  }

  if ((!(diffab > 0.0)) || (!(diffab < 3.1415926535897931))) {
    diffab -= 6.2831853071795862;
  }

  return diffab;
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static real_T RflysimIndoorController_r2_norm(const real_T x[2])
{
  real_T y;
  real_T t;
  RflysimIndoorController_r2018_B.scale = 3.3121686421112381E-170;
  RflysimIndoorController_r2018_B.absxk = fabs(x[0]);
  if (RflysimIndoorController_r2018_B.absxk > 3.3121686421112381E-170) {
    y = 1.0;
    RflysimIndoorController_r2018_B.scale =
      RflysimIndoorController_r2018_B.absxk;
  } else {
    t = RflysimIndoorController_r2018_B.absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  RflysimIndoorController_r2018_B.absxk = fabs(x[1]);
  if (RflysimIndoorController_r2018_B.absxk >
      RflysimIndoorController_r2018_B.scale) {
    t = RflysimIndoorController_r2018_B.scale /
      RflysimIndoorController_r2018_B.absxk;
    y = y * t * t + 1.0;
    RflysimIndoorController_r2018_B.scale =
      RflysimIndoorController_r2018_B.absxk;
  } else {
    t = RflysimIndoorController_r2018_B.absxk /
      RflysimIndoorController_r2018_B.scale;
    y += t * t;
  }

  return RflysimIndoorController_r2018_B.scale * sqrt(y);
}

static real_T RflysimIndoorContro_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    RflysimIndoorController_r2018_B.d0 = fabs(u0);
    RflysimIndoorController_r2018_B.d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (RflysimIndoorController_r2018_B.d0 == 1.0) {
        y = 1.0;
      } else if (RflysimIndoorController_r2018_B.d0 > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (RflysimIndoorController_r2018_B.d1 == 0.0) {
      y = 1.0;
    } else if (RflysimIndoorController_r2018_B.d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static real_T RflysimIndoorControll_dmysigma2(real_T x, real_T d1, real_T d2)
{
  real_T u;
  if (x <= d1) {
    u = 1.0;
  } else if ((d1 <= x) && (x <= d2)) {
    RflysimIndoorController_r2018_B.u_tmp_tmp_f = d1 - d2;
    RflysimIndoorController_r2018_B.u_tmp = RflysimIndoorContro_rt_powd_snf
      (RflysimIndoorController_r2018_B.u_tmp_tmp_f, 3.0);
    RflysimIndoorController_r2018_B.u_tmp_tmp_c = d1 * d1;
    RflysimIndoorController_r2018_B.u_tmp_c =
      RflysimIndoorController_r2018_B.u_tmp_tmp_c * -0.0;
    RflysimIndoorController_r2018_B.u_tmp_b = 0.0 * d1 * d2;
    RflysimIndoorController_r2018_B.u_tmp_tmp = d2 * d2;
    RflysimIndoorController_r2018_B.u_tmp_p =
      RflysimIndoorController_r2018_B.u_tmp_tmp * 0.0;
    u = ((((((RflysimIndoorController_r2018_B.u_tmp_c -
              RflysimIndoorController_r2018_B.u_tmp_b) + 3.0 * d1) +
            RflysimIndoorController_r2018_B.u_tmp_p) + 3.0 * d2) /
          RflysimIndoorController_r2018_B.u_tmp * (x * x) + -((d2 * 0.0 - d1 *
            0.0) + 2.0) / RflysimIndoorController_r2018_B.u_tmp *
          RflysimIndoorContro_rt_powd_snf(x, 3.0)) +
         -((((RflysimIndoorController_r2018_B.u_tmp_c +
              RflysimIndoorController_r2018_B.u_tmp_b) + 6.0 * d1) +
            RflysimIndoorController_r2018_B.u_tmp_p) * d2) /
         RflysimIndoorController_r2018_B.u_tmp * x) +
      (((RflysimIndoorController_r2018_B.u_tmp_c * d2 + 0.0 * d1 *
         RflysimIndoorController_r2018_B.u_tmp_tmp) + 3.0 * d1 * d2) -
       RflysimIndoorController_r2018_B.u_tmp_tmp) * d2 /
      (((RflysimIndoorController_r2018_B.u_tmp_tmp_c - 2.0 * d1 * d2) +
        RflysimIndoorController_r2018_B.u_tmp_tmp) *
       RflysimIndoorController_r2018_B.u_tmp_tmp_f);
  } else {
    u = 0.0;
  }

  return u;
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static real_T RflysimIndoorController_r20_dot(const real_T a[2], const real_T b
  [2])
{
  return a[0] * b[0] + a[1] * b[1];
}

// Function for MATLAB Function: '<Root>/MATLAB Function2'
static void RflysimIndoorCont_Encirclement4(const real_T p_leader[1000], const
  real_T tunnel_left[1000], const real_T tunnel_right[1000], const real_T
  p_target[2], const real_T Pcur[24], const real_T Vcur[24], const real_T
  Yawcur[12], real_T gain, real_T vmax, real_T rs, real_T ra, real_T rt1, real_T
  rt2, real_T i, real_T u[2], real_T *ux, real_T *uy, real_T *uyaw, real_T
  *leader_locate, real_T *dist)
{
  int32_T j;
  int32_T i_tmp;
  int32_T ep_tmp;
  i_tmp = (int32_T)i;
  *uyaw = RflysimIndoorContr_minAngleDiff(rt_atan2d_snf(p_target[1] - Pcur[i_tmp
    + 11], p_target[0] - Pcur[i_tmp - 1]), Yawcur[(int32_T)i - 1]);
  *leader_locate = 0.0;
  RflysimIndoorController_r2018_B.dis_leader_min = 1.0E+8;
  for (j = 0; j < 500; j++) {
    RflysimIndoorController_r2018_B.Pcur_k[0] = Pcur[i_tmp - 1] - p_leader[j];
    RflysimIndoorController_r2018_B.Pcur_k[1] = Pcur[i_tmp + 11] - p_leader[j +
      500];
    RflysimIndoorController_r2018_B.dis_leader = RflysimIndoorController_r2_norm
      (RflysimIndoorController_r2018_B.Pcur_k);
    if (RflysimIndoorController_r2018_B.dis_leader <
        RflysimIndoorController_r2018_B.dis_leader_min) {
      *leader_locate = 1.0 + (real_T)j;
      RflysimIndoorController_r2018_B.dis_leader_min =
        RflysimIndoorController_r2018_B.dis_leader;
    }

    RflysimIndoorController_r2018_B.dis_leader = tunnel_right[j] - tunnel_left[j];
    RflysimIndoorController_r2018_B.rtksii = tunnel_right[j + 500] -
      tunnel_left[j + 500];
    RflysimIndoorController_r2018_B.ep[0] = 0.0 *
      RflysimIndoorController_r2018_B.dis_leader +
      -RflysimIndoorController_r2018_B.rtksii;
    RflysimIndoorController_r2018_B.ep[1] = 0.0 *
      RflysimIndoorController_r2018_B.rtksii +
      RflysimIndoorController_r2018_B.dis_leader;
    RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r2_norm
      (RflysimIndoorController_r2018_B.ep);
    RflysimIndoorController_r2018_B.nc[j] =
      -(RflysimIndoorController_r2018_B.ep[0] /
        RflysimIndoorController_r2018_B.rtksii);
    RflysimIndoorController_r2018_B.nc[j + 500] =
      -(RflysimIndoorController_r2018_B.ep[1] /
        RflysimIndoorController_r2018_B.rtksii);
  }

  RflysimIndoorController_r2018_B.V1[0] = 0.0;
  RflysimIndoorController_r2018_B.dis_leader_min = 0.0;
  RflysimIndoorController_r2018_B.V3[0] = 0.0;
  RflysimIndoorController_r2018_B.V1[1] = 0.0;
  RflysimIndoorController_r2018_B.dis_leader = 0.0;
  RflysimIndoorController_r2018_B.V3[1] = 0.0;
  *dist = 100000.0;
  if (*leader_locate != 0.0) {
    j = (int32_T)*leader_locate - 1;
    RflysimIndoorController_r2018_B.ep[0] = RflysimIndoorController_r2018_B.nc[j]
      * vmax;
    ep_tmp = (int32_T)*leader_locate + 499;
    RflysimIndoorController_r2018_B.ep[1] =
      RflysimIndoorController_r2018_B.nc[ep_tmp] * vmax;
    RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r2_norm
      (RflysimIndoorController_r2018_B.ep);
    if (RflysimIndoorController_r2018_B.rtksii > vmax) {
      RflysimIndoorController_r2018_B.V1[0] = vmax *
        RflysimIndoorController_r2018_B.ep[0] /
        RflysimIndoorController_r2018_B.rtksii;
      RflysimIndoorController_r2018_B.V1[1] = vmax *
        RflysimIndoorController_r2018_B.ep[1] /
        RflysimIndoorController_r2018_B.rtksii;
    } else {
      RflysimIndoorController_r2018_B.V1[0] =
        RflysimIndoorController_r2018_B.ep[0];
      RflysimIndoorController_r2018_B.V1[1] =
        RflysimIndoorController_r2018_B.ep[1];
    }

    RflysimIndoorController_r2018_B.rtksii = 1.0 / gain;
    RflysimIndoorController_r2018_B.disti = (tunnel_left[j] + tunnel_right[j]) *
      0.5;
    RflysimIndoorController_r2018_B.V3[0] = (Vcur[i_tmp - 1] *
      RflysimIndoorController_r2018_B.rtksii + Pcur[i_tmp - 1]) -
      RflysimIndoorController_r2018_B.disti;
    RflysimIndoorController_r2018_B.Pcur_k[0] = tunnel_left[(int32_T)
      *leader_locate - 1] - tunnel_right[(int32_T)*leader_locate - 1];
    RflysimIndoorController_r2018_B.V1[0] = -RflysimIndoorController_r2018_B.V1
      [0];
    RflysimIndoorController_r2018_B.ep[0] =
      RflysimIndoorController_r2018_B.disti;
    RflysimIndoorController_r2018_B.disti = (tunnel_left[ep_tmp] +
      tunnel_right[ep_tmp]) * 0.5;
    RflysimIndoorController_r2018_B.V3[1] = (Vcur[i_tmp + 11] *
      RflysimIndoorController_r2018_B.rtksii + Pcur[i_tmp + 11]) -
      RflysimIndoorController_r2018_B.disti;
    RflysimIndoorController_r2018_B.Pcur_k[1] = tunnel_left[(int32_T)
      *leader_locate + 499] - tunnel_right[(int32_T)*leader_locate + 499];
    RflysimIndoorController_r2018_B.V1[1] = -RflysimIndoorController_r2018_B.V1
      [1];
    RflysimIndoorController_r2018_B.rtksii = 0.5 *
      RflysimIndoorController_r2_norm(RflysimIndoorController_r2018_B.Pcur_k);
    RflysimIndoorController_r2018_B.Pcur_k[0] = Pcur[i_tmp - 1] -
      RflysimIndoorController_r2018_B.ep[0];
    RflysimIndoorController_r2018_B.Pcur_k[1] = Pcur[i_tmp + 11] -
      RflysimIndoorController_r2018_B.disti;
    RflysimIndoorController_r2018_B.disti =
      RflysimIndoorController_r2018_B.rtksii - RflysimIndoorController_r2_norm
      (RflysimIndoorController_r2018_B.Pcur_k);
    if (RflysimIndoorController_r2018_B.disti < 100000.0) {
      *dist = RflysimIndoorController_r2018_B.disti;
    }

    RflysimIndoorController_r2018_B.disti = RflysimIndoorController_r2_norm
      (RflysimIndoorController_r2018_B.V3);
    RflysimIndoorController_r2018_B.dVtidksiti = 3.0 * vmax *
      RflysimIndoorControll_dmysigma2(RflysimIndoorController_r2018_B.rtksii -
      RflysimIndoorController_r2018_B.disti, rt1, rt2);
    RflysimIndoorController_r2018_B.Pcur_k[0] =
      RflysimIndoorController_r2018_B.nc[(int32_T)*leader_locate - 1];
    RflysimIndoorController_r2018_B.Pcur_k[1] =
      RflysimIndoorController_r2018_B.nc[(int32_T)*leader_locate + 499];
    RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r20_dot
      (RflysimIndoorController_r2018_B.V3,
       RflysimIndoorController_r2018_B.Pcur_k);
    RflysimIndoorController_r2018_B.ep[0] = RflysimIndoorController_r2018_B.V3[0]
      - RflysimIndoorController_r2018_B.nc[(int32_T)*leader_locate - 1] *
      RflysimIndoorController_r2018_B.rtksii;
    RflysimIndoorController_r2018_B.ep[1] = RflysimIndoorController_r2018_B.V3[1]
      - RflysimIndoorController_r2018_B.nc[(int32_T)*leader_locate + 499] *
      RflysimIndoorController_r2018_B.rtksii;
    RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r2_norm
      (RflysimIndoorController_r2018_B.ep);
    RflysimIndoorController_r2018_B.b_tmp[0] =
      RflysimIndoorController_r2018_B.ep[0] /
      RflysimIndoorController_r2018_B.rtksii;
    RflysimIndoorController_r2018_B.Pcur_k[0] =
      RflysimIndoorController_r2018_B.dVtidksiti *
      RflysimIndoorController_r2018_B.V3[0] /
      RflysimIndoorController_r2018_B.disti;
    RflysimIndoorController_r2018_B.b_tmp[1] =
      RflysimIndoorController_r2018_B.ep[1] /
      RflysimIndoorController_r2018_B.rtksii;
    RflysimIndoorController_r2018_B.Pcur_k[1] =
      RflysimIndoorController_r2018_B.dVtidksiti *
      RflysimIndoorController_r2018_B.V3[1] /
      RflysimIndoorController_r2018_B.disti;
    RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r20_dot
      (RflysimIndoorController_r2018_B.b_tmp,
       RflysimIndoorController_r2018_B.Pcur_k);
    RflysimIndoorController_r2018_B.V3[0] =
      -RflysimIndoorController_r2018_B.b_tmp[0] *
      RflysimIndoorController_r2018_B.rtksii;
    RflysimIndoorController_r2018_B.V3[1] =
      -RflysimIndoorController_r2018_B.b_tmp[1] *
      RflysimIndoorController_r2018_B.rtksii;
    for (j = 0; j < 3; j++) {
      if (1.0 + (real_T)j != i) {
        RflysimIndoorController_r2018_B.rtksii = 1.0 / gain;
        RflysimIndoorController_r2018_B.ep[0] = (Vcur[i_tmp - 1] - Vcur[j]) *
          RflysimIndoorController_r2018_B.rtksii + (Pcur[i_tmp - 1] - Pcur[j]);
        RflysimIndoorController_r2018_B.ep[1] = (Vcur[i_tmp + 11] - Vcur[j + 12])
          * RflysimIndoorController_r2018_B.rtksii + (Pcur[i_tmp + 11] - Pcur[j
          + 12]);
        RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r2_norm
          (RflysimIndoorController_r2018_B.ep);
        RflysimIndoorController_r2018_B.disti = 20.0 * vmax *
          RflysimIndoorControll_dmysigma2(RflysimIndoorController_r2018_B.rtksii,
          2.0 * rs, rs + ra);
        RflysimIndoorController_r2018_B.dis_leader_min +=
          RflysimIndoorController_r2018_B.ep[0] /
          RflysimIndoorController_r2018_B.rtksii *
          RflysimIndoorController_r2018_B.disti;
        RflysimIndoorController_r2018_B.dis_leader +=
          RflysimIndoorController_r2018_B.ep[1] /
          RflysimIndoorController_r2018_B.rtksii *
          RflysimIndoorController_r2018_B.disti;
      }
    }
  }

  RflysimIndoorController_r2018_B.V1[0] = (RflysimIndoorController_r2018_B.V1[0]
    + RflysimIndoorController_r2018_B.dis_leader_min) +
    RflysimIndoorController_r2018_B.V3[0];
  RflysimIndoorController_r2018_B.dis_leader_min =
    (RflysimIndoorController_r2018_B.V1[1] +
     RflysimIndoorController_r2018_B.dis_leader) +
    RflysimIndoorController_r2018_B.V3[1];
  RflysimIndoorController_r2018_B.V1[1] =
    RflysimIndoorController_r2018_B.dis_leader_min;
  RflysimIndoorController_r2018_B.rtksii = RflysimIndoorController_r2_norm
    (RflysimIndoorController_r2018_B.V1);
  if (RflysimIndoorController_r2018_B.rtksii > vmax) {
    u[0] = vmax * RflysimIndoorController_r2018_B.V1[0] /
      RflysimIndoorController_r2018_B.rtksii;
    u[1] = vmax * RflysimIndoorController_r2018_B.dis_leader_min /
      RflysimIndoorController_r2018_B.rtksii;
  } else {
    u[0] = RflysimIndoorController_r2018_B.V1[0];
    u[1] = RflysimIndoorController_r2018_B.dis_leader_min;
  }

  *ux = u[0];
  *uy = u[1];
}

static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void RflysimIndoo_matlabCodegenHa_i1(robotics_slros_internal_blo_j_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void RflysimIndoorController_r2018b_N12_V9_step(void)
{
  boolean_T b_varargout_1;
  int32_T rtb_tri4;
  int32_T rtb_tri3;
  int32_T rtb_tri2;
  int32_T j;
  SL_Bus_RflysimIndoorController_r_Pipeunit_6gaofl *tunnel_msg_Units;

  // Outputs for Atomic SubSystem: '<S17>/Subscribe'
  // MATLABSystem: '<S104>/SourceBlock' incorporates:
  //   Inport: '<S116>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15717.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S104>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S116>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_ks =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S104>/SourceBlock'
  // End of Outputs for SubSystem: '<S104>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe4'
  // MATLABSystem: '<S110>/SourceBlock' incorporates:
  //   Inport: '<S122>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15833.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S110>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S122>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_h3 =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S110>/SourceBlock'
  // End of Outputs for SubSystem: '<S110>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe4'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe8'
  // MATLABSystem: '<S114>/SourceBlock' incorporates:
  //   Inport: '<S126>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15851.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S114>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S126>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_lp =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S114>/SourceBlock'
  // End of Outputs for SubSystem: '<S114>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe8'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe1'
  // MATLABSystem: '<S105>/SourceBlock' incorporates:
  //   Inport: '<S117>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15920.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S105>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S117>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_n0 =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S105>/SourceBlock'
  // End of Outputs for SubSystem: '<S105>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe1'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe2'
  // MATLABSystem: '<S108>/SourceBlock' incorporates:
  //   Inport: '<S120>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15921.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S108>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S120>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_ox =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S108>/SourceBlock'
  // End of Outputs for SubSystem: '<S108>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe2'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe3'
  // MATLABSystem: '<S109>/SourceBlock' incorporates:
  //   Inport: '<S121>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15922.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S109>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S121>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_dj =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S109>/SourceBlock'
  // End of Outputs for SubSystem: '<S109>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe3'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe5'
  // MATLABSystem: '<S111>/SourceBlock' incorporates:
  //   Inport: '<S123>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16033.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S111>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S123>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_hr =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S111>/SourceBlock'
  // End of Outputs for SubSystem: '<S111>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe5'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe10'
  // MATLABSystem: '<S106>/SourceBlock' incorporates:
  //   Inport: '<S118>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16037.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S106>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S118>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_fk =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S106>/SourceBlock'
  // End of Outputs for SubSystem: '<S106>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe10'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe11'
  // MATLABSystem: '<S107>/SourceBlock' incorporates:
  //   Inport: '<S119>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16038.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S107>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S119>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_er =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S107>/SourceBlock'
  // End of Outputs for SubSystem: '<S107>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe11'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe6'
  // MATLABSystem: '<S112>/SourceBlock' incorporates:
  //   Inport: '<S124>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16034.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S112>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S124>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_o =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S112>/SourceBlock'
  // End of Outputs for SubSystem: '<S112>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe6'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe7'
  // MATLABSystem: '<S113>/SourceBlock' incorporates:
  //   Inport: '<S125>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16035.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S113>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S125>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_b1 =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S113>/SourceBlock'
  // End of Outputs for SubSystem: '<S113>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe7'

  // Outputs for Atomic SubSystem: '<S17>/Subscribe9'
  // MATLABSystem: '<S115>/SourceBlock' incorporates:
  //   Inport: '<S127>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16036.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S115>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S127>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_a1 =
      RflysimIndoorController_r2018_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S115>/SourceBlock'
  // End of Outputs for SubSystem: '<S115>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S17>/Subscribe9'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe3'
  // MATLABSystem: '<S23>/SourceBlock' incorporates:
  //   Inport: '<S133>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15807.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S23>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S133>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_g =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S23>/SourceBlock'
  // End of Outputs for SubSystem: '<S23>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe3'

  // MATLAB Function: '<Root>/MATLAB Function4'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_g,
    &RflysimIndoorController_r201_DW.Memory4_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_h);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe7'
  // MATLABSystem: '<S27>/SourceBlock' incorporates:
  //   Inport: '<S137>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15836.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S27>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S137>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_f =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S27>/SourceBlock'
  // End of Outputs for SubSystem: '<S27>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe7'

  // MATLAB Function: '<Root>/MATLAB Function8'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_f,
    &RflysimIndoorController_r201_DW.Memory5_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_j);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe11'
  // MATLABSystem: '<S20>/SourceBlock' incorporates:
  //   Inport: '<S130>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15854.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S20>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S130>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_n =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S20>/SourceBlock'
  // End of Outputs for SubSystem: '<S20>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe11'

  // MATLAB Function: '<Root>/MATLAB Function10'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_n,
    &RflysimIndoorController_r201_DW.Memory6_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_mz);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe2'
  // MATLABSystem: '<S22>/SourceBlock' incorporates:
  //   Inport: '<S132>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15933.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S22>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S132>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_m =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S22>/SourceBlock'
  // End of Outputs for SubSystem: '<S22>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe2'

  // MATLAB Function: '<Root>/MATLAB Function6'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_m,
    &RflysimIndoorController_r201_DW.Memory1_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_f);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe4'
  // MATLABSystem: '<S24>/SourceBlock' incorporates:
  //   Inport: '<S134>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15934.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S24>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S134>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_l =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S24>/SourceBlock'
  // End of Outputs for SubSystem: '<S24>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe4'

  // MATLAB Function: '<Root>/MATLAB Function7'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_l,
    &RflysimIndoorController_r201_DW.Memory2_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_o);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S18>/SourceBlock' incorporates:
  //   Inport: '<S128>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15932.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S18>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S128>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_d =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S18>/SourceBlock'
  // End of Outputs for SubSystem: '<S18>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // MATLAB Function: '<Root>/MATLAB Function5'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_d,
    &RflysimIndoorController_r201_DW.Memory3_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_i);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe9'
  // MATLABSystem: '<S29>/SourceBlock' incorporates:
  //   Inport: '<S139>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16018.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S29>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S139>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1 =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S29>/SourceBlock'
  // End of Outputs for SubSystem: '<S29>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe9'

  // MATLAB Function: '<Root>/MATLAB Function11'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1,
    &RflysimIndoorController_r201_DW.Memory10_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_b);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe12'
  // MATLABSystem: '<S21>/SourceBlock' incorporates:
  //   Inport: '<S131>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16020.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S21>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S131>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_p =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S21>/SourceBlock'
  // End of Outputs for SubSystem: '<S21>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe12'

  // MATLAB Function: '<Root>/MATLAB Function15'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_p,
    &RflysimIndoorController_r201_DW.Memory11_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_p);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe6'
  // MATLABSystem: '<S26>/SourceBlock' incorporates:
  //   Inport: '<S136>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16016.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S136>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_a4 =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S26>/SourceBlock'
  // End of Outputs for SubSystem: '<S26>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe6'

  // MATLAB Function: '<Root>/MATLAB Function9'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_a4,
    &RflysimIndoorController_r201_DW.Memory12_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe8'
  // MATLABSystem: '<S28>/SourceBlock' incorporates:
  //   Inport: '<S138>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16017.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S138>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_a =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S28>/SourceBlock'
  // End of Outputs for SubSystem: '<S28>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe8'

  // MATLAB Function: '<Root>/MATLAB Function13'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_a,
    &RflysimIndoorController_r201_DW.Memory7_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_m);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe10'
  // MATLABSystem: '<S19>/SourceBlock' incorporates:
  //   Inport: '<S129>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16019.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S19>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S129>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_k =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S19>/SourceBlock'
  // End of Outputs for SubSystem: '<S19>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe10'

  // MATLAB Function: '<Root>/MATLAB Function14'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_k,
    &RflysimIndoorController_r201_DW.Memory8_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_pf);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe5'
  // MATLABSystem: '<S25>/SourceBlock' incorporates:
  //   Inport: '<S135>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16015.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S135>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_h =
      RflysimIndoorController_r2018_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S25>/SourceBlock'
  // End of Outputs for SubSystem: '<S25>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe5'

  // MATLAB Function: '<Root>/MATLAB Function12'
  RflysimIndoorC_MATLABFunction10(&RflysimIndoorController_r2018_B.In1_h,
    &RflysimIndoorController_r201_DW.Memory9_PreviousInput,
    &RflysimIndoorController_r2018_B.msg_new_m2);

  // Outputs for Atomic SubSystem: '<S16>/Subscribe1'
  // MATLABSystem: '<S56>/SourceBlock' incorporates:
  //   Inport: '<S80>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15722.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S56>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S80>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_a4x =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S56>/SourceBlock'
  // End of Outputs for SubSystem: '<S56>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe1'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe5'
  // MATLABSystem: '<S75>/SourceBlock' incorporates:
  //   Inport: '<S99>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15834.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S75>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S99>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_lm =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S75>/SourceBlock'
  // End of Outputs for SubSystem: '<S75>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe5'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe9'
  // MATLABSystem: '<S79>/SourceBlock' incorporates:
  //   Inport: '<S103>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15852.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S79>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S103>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_b =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S79>/SourceBlock'
  // End of Outputs for SubSystem: '<S79>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe9'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe3'
  // MATLABSystem: '<S73>/SourceBlock' incorporates:
  //   Inport: '<S97>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15947.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S73>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S97>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_hw =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S73>/SourceBlock'
  // End of Outputs for SubSystem: '<S73>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe3'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe4'
  // MATLABSystem: '<S74>/SourceBlock' incorporates:
  //   Inport: '<S98>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15948.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S74>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S98>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_j =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S74>/SourceBlock'
  // End of Outputs for SubSystem: '<S74>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe4'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe7'
  // MATLABSystem: '<S77>/SourceBlock' incorporates:
  //   Inport: '<S101>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15949.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S77>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S101>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_fh =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S77>/SourceBlock'
  // End of Outputs for SubSystem: '<S77>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe7'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe13'
  // MATLABSystem: '<S60>/SourceBlock' incorporates:
  //   Inport: '<S84>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16063.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S60>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S84>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_fo =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S60>/SourceBlock'
  // End of Outputs for SubSystem: '<S60>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe13'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe16'
  // MATLABSystem: '<S63>/SourceBlock' incorporates:
  //   Inport: '<S87>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16066.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S63>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S87>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_nu =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S63>/SourceBlock'
  // End of Outputs for SubSystem: '<S63>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe16'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe18'
  // MATLABSystem: '<S65>/SourceBlock' incorporates:
  //   Inport: '<S89>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16068.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S65>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S89>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_bp =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S65>/SourceBlock'
  // End of Outputs for SubSystem: '<S65>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe18'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe14'
  // MATLABSystem: '<S61>/SourceBlock' incorporates:
  //   Inport: '<S85>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16064.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S61>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S85>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_ba =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S61>/SourceBlock'
  // End of Outputs for SubSystem: '<S61>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe14'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe15'
  // MATLABSystem: '<S62>/SourceBlock' incorporates:
  //   Inport: '<S86>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16065.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S62>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S86>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_dh =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S62>/SourceBlock'
  // End of Outputs for SubSystem: '<S62>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe15'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe17'
  // MATLABSystem: '<S64>/SourceBlock' incorporates:
  //   Inport: '<S88>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16067.getLatestMessage
    (&RflysimIndoorController_r2018_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<S64>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S88>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_mu =
      RflysimIndoorController_r2018_B.b_varargout_2_m;
  }

  // End of MATLABSystem: '<S64>/SourceBlock'
  // End of Outputs for SubSystem: '<S64>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe17'

  // MATLAB Function: '<S16>/MATLAB Function11'
  RflysimIndoorController_r2018_B.Pcur[0] =
    RflysimIndoorController_r2018_B.In1_a4x.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[12] =
    RflysimIndoorController_r2018_B.In1_a4x.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[1] =
    RflysimIndoorController_r2018_B.In1_lm.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[13] =
    RflysimIndoorController_r2018_B.In1_lm.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[2] =
    RflysimIndoorController_r2018_B.In1_b.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[14] =
    RflysimIndoorController_r2018_B.In1_b.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[3] =
    RflysimIndoorController_r2018_B.In1_hw.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[15] =
    RflysimIndoorController_r2018_B.In1_hw.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[4] =
    RflysimIndoorController_r2018_B.In1_j.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[16] =
    RflysimIndoorController_r2018_B.In1_j.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[5] =
    RflysimIndoorController_r2018_B.In1_fh.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[17] =
    RflysimIndoorController_r2018_B.In1_fh.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[6] =
    RflysimIndoorController_r2018_B.In1_fo.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[18] =
    RflysimIndoorController_r2018_B.In1_fo.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[7] =
    RflysimIndoorController_r2018_B.In1_nu.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[19] =
    RflysimIndoorController_r2018_B.In1_nu.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[8] =
    RflysimIndoorController_r2018_B.In1_bp.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[20] =
    RflysimIndoorController_r2018_B.In1_bp.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[9] =
    RflysimIndoorController_r2018_B.In1_ba.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[21] =
    RflysimIndoorController_r2018_B.In1_ba.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[10] =
    RflysimIndoorController_r2018_B.In1_dh.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[22] =
    RflysimIndoorController_r2018_B.In1_dh.Pose.Position.Y;
  RflysimIndoorController_r2018_B.Pcur[11] =
    RflysimIndoorController_r2018_B.In1_mu.Pose.Position.X;
  RflysimIndoorController_r2018_B.Pcur[23] =
    RflysimIndoorController_r2018_B.In1_mu.Pose.Position.Y;

  // Outputs for Atomic SubSystem: '<S16>/Subscribe2'
  // MATLABSystem: '<S67>/SourceBlock' incorporates:
  //   Inport: '<S91>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15777.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S67>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S91>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_jl =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S67>/SourceBlock'
  // End of Outputs for SubSystem: '<S67>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe2'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe6'
  // MATLABSystem: '<S76>/SourceBlock' incorporates:
  //   Inport: '<S100>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15835.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S76>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S100>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_m1 =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S76>/SourceBlock'
  // End of Outputs for SubSystem: '<S76>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe6'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe10'
  // MATLABSystem: '<S57>/SourceBlock' incorporates:
  //   Inport: '<S81>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15853.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S57>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S81>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_jk =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S57>/SourceBlock'
  // End of Outputs for SubSystem: '<S57>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe10'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe11'
  // MATLABSystem: '<S58>/SourceBlock' incorporates:
  //   Inport: '<S82>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15957.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S58>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S82>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_mhe =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S58>/SourceBlock'
  // End of Outputs for SubSystem: '<S58>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe11'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe12'
  // MATLABSystem: '<S59>/SourceBlock' incorporates:
  //   Inport: '<S83>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15958.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S59>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S83>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_e =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S59>/SourceBlock'
  // End of Outputs for SubSystem: '<S59>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe12'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe8'
  // MATLABSystem: '<S78>/SourceBlock' incorporates:
  //   Inport: '<S102>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_15956.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S78>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S102>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_dh3 =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S78>/SourceBlock'
  // End of Outputs for SubSystem: '<S78>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe8'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe22'
  // MATLABSystem: '<S70>/SourceBlock' incorporates:
  //   Inport: '<S94>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16102.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S70>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S94>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_mh =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S70>/SourceBlock'
  // End of Outputs for SubSystem: '<S70>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe22'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe23'
  // MATLABSystem: '<S71>/SourceBlock' incorporates:
  //   Inport: '<S95>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16103.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S71>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S95>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_hk =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S71>/SourceBlock'
  // End of Outputs for SubSystem: '<S71>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe23'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe19'
  // MATLABSystem: '<S66>/SourceBlock' incorporates:
  //   Inport: '<S90>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16099.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S66>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S90>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_he =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S66>/SourceBlock'
  // End of Outputs for SubSystem: '<S66>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe19'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe20'
  // MATLABSystem: '<S68>/SourceBlock' incorporates:
  //   Inport: '<S92>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16100.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S68>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S92>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_ha =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S68>/SourceBlock'
  // End of Outputs for SubSystem: '<S68>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe20'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe21'
  // MATLABSystem: '<S69>/SourceBlock' incorporates:
  //   Inport: '<S93>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16101.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S69>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S93>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_fz =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S69>/SourceBlock'
  // End of Outputs for SubSystem: '<S69>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe21'

  // Outputs for Atomic SubSystem: '<S16>/Subscribe24'
  // MATLABSystem: '<S72>/SourceBlock' incorporates:
  //   Inport: '<S96>/In1'

  b_varargout_1 =
    Sub_RflysimIndoorController_r2018b_N12_V9_16104.getLatestMessage
    (&RflysimIndoorController_r2018_B.BusAssignment1_j);

  // Outputs for Enabled SubSystem: '<S72>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S96>/Enable'

  if (b_varargout_1) {
    RflysimIndoorController_r2018_B.In1_ai =
      RflysimIndoorController_r2018_B.BusAssignment1_j;
  }

  // End of MATLABSystem: '<S72>/SourceBlock'
  // End of Outputs for SubSystem: '<S72>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S16>/Subscribe24'

  // MATLAB Function: '<S16>/MATLAB Function12'
  RflysimIndoorController_r2018_B.Vcur[0] =
    RflysimIndoorController_r2018_B.In1_jl.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[12] =
    RflysimIndoorController_r2018_B.In1_jl.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[1] =
    RflysimIndoorController_r2018_B.In1_m1.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[13] =
    RflysimIndoorController_r2018_B.In1_m1.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[2] =
    RflysimIndoorController_r2018_B.In1_jk.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[14] =
    RflysimIndoorController_r2018_B.In1_jk.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[3] =
    RflysimIndoorController_r2018_B.In1_mhe.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[15] =
    RflysimIndoorController_r2018_B.In1_mhe.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[4] =
    RflysimIndoorController_r2018_B.In1_e.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[16] =
    RflysimIndoorController_r2018_B.In1_e.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[5] =
    RflysimIndoorController_r2018_B.In1_dh3.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[17] =
    RflysimIndoorController_r2018_B.In1_dh3.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[6] =
    RflysimIndoorController_r2018_B.In1_mh.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[18] =
    RflysimIndoorController_r2018_B.In1_mh.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[7] =
    RflysimIndoorController_r2018_B.In1_hk.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[19] =
    RflysimIndoorController_r2018_B.In1_hk.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[8] =
    RflysimIndoorController_r2018_B.In1_he.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[20] =
    RflysimIndoorController_r2018_B.In1_he.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[9] =
    RflysimIndoorController_r2018_B.In1_ha.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[21] =
    RflysimIndoorController_r2018_B.In1_ha.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[10] =
    RflysimIndoorController_r2018_B.In1_fz.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[22] =
    RflysimIndoorController_r2018_B.In1_fz.Twist.Linear.Y;
  RflysimIndoorController_r2018_B.Vcur[11] =
    RflysimIndoorController_r2018_B.In1_ai.Twist.Linear.X;
  RflysimIndoorController_r2018_B.Vcur[23] =
    RflysimIndoorController_r2018_B.In1_ai.Twist.Linear.Y;

  // SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport18' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function2'
  //   MATLAB Function: '<S16>/MATLAB Function'
  //   MATLAB Function: '<S16>/MATLAB Function1'
  //   MATLAB Function: '<S16>/MATLAB Function10'
  //   MATLAB Function: '<S16>/MATLAB Function13'
  //   MATLAB Function: '<S16>/MATLAB Function2'
  //   MATLAB Function: '<S16>/MATLAB Function3'
  //   MATLAB Function: '<S16>/MATLAB Function4'
  //   MATLAB Function: '<S16>/MATLAB Function5'
  //   MATLAB Function: '<S16>/MATLAB Function6'
  //   MATLAB Function: '<S16>/MATLAB Function7'
  //   MATLAB Function: '<S16>/MATLAB Function8'
  //   MATLAB Function: '<S16>/MATLAB Function9'

  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_a4x.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[0]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_lm.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[1]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_b.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[2]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_hw.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[3]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_j.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[4]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_fh.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[5]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_fo.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[6]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_nu.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[7]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_bp.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[8]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_ba.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[9]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_dh.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[10]);
  RflysimIndoorCon_MATLABFunction
    (&RflysimIndoorController_r2018_B.In1_mu.Pose.Orientation,
     &RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p[11]);

  // SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function2'
  //   MATLAB Function: '<Root>/MATLAB Function3'
  //   SignalConversion: '<S9>/TmpSignal ConversionAt SFunction Inport1'

  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[0] =
    RflysimIndoorController_r2018_B.In1_ks.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[1] =
    RflysimIndoorController_r2018_B.In1_h3.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[2] =
    RflysimIndoorController_r2018_B.In1_lp.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[3] =
    RflysimIndoorController_r2018_B.In1_n0.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[4] =
    RflysimIndoorController_r2018_B.In1_ox.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[5] =
    RflysimIndoorController_r2018_B.In1_dj.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[6] =
    RflysimIndoorController_r2018_B.In1_hr.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[7] =
    RflysimIndoorController_r2018_B.In1_fk.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[8] =
    RflysimIndoorController_r2018_B.In1_er.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[9] =
    RflysimIndoorController_r2018_B.In1_o.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[10] =
    RflysimIndoorController_r2018_B.In1_b1.Data;
  RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[11] =
    RflysimIndoorController_r2018_B.In1_a1.Data;

  // SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport17' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function2'

  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[0] =
    RflysimIndoorController_r2018_B.In1_a4x.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[1] =
    RflysimIndoorController_r2018_B.In1_lm.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[2] =
    RflysimIndoorController_r2018_B.In1_b.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[3] =
    RflysimIndoorController_r2018_B.In1_hw.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[4] =
    RflysimIndoorController_r2018_B.In1_j.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[5] =
    RflysimIndoorController_r2018_B.In1_fh.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[6] =
    RflysimIndoorController_r2018_B.In1_fo.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[7] =
    RflysimIndoorController_r2018_B.In1_nu.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[8] =
    RflysimIndoorController_r2018_B.In1_bp.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[9] =
    RflysimIndoorController_r2018_B.In1_ba.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[10] =
    RflysimIndoorController_r2018_B.In1_dh.Pose.Position.Z;
  RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[11] =
    RflysimIndoorController_r2018_B.In1_mu.Pose.Position.Z;

  // MATLAB Function: '<Root>/MATLAB Function2' incorporates:
  //   Constant: '<Root>/Constant1'
  //   Constant: '<Root>/Constant10'
  //   Constant: '<Root>/Constant2'
  //   Constant: '<Root>/Constant5'
  //   Constant: '<Root>/Constant7'
  //   Constant: '<Root>/Constant8'
  //   Constant: '<Root>/Constant9'
  //   SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1'

  for (j = 0; j < 12; j++) {
    RflysimIndoorController_r2018_B.ux[j] = 0.0;
    RflysimIndoorController_r2018_B.uy[j] = 0.0;
    RflysimIndoorController_r2018_B.uyaw[j] = 0.0;
    RflysimIndoorController_r2018_B.states_new[j] = 0.0;
    RflysimIndoorController_r2018_B.uz[j] = RflysimIndoorControlle_mysatNum((4.0
      - RflysimIndoorController_r2018_B.TmpSignalConversionAtSFunct[j]) * 0.8);
    memset(&RflysimIndoorController_r2018_B.left_x[0], 0, 10U * sizeof(real_T));
    memset(&RflysimIndoorController_r2018_B.left_y[0], 0, 10U * sizeof(real_T));
    memset(&RflysimIndoorController_r2018_B.right_x[0], 0, 10U * sizeof(real_T));
    memset(&RflysimIndoorController_r2018_B.right_y[0], 0, 10U * sizeof(real_T));
    memset(&RflysimIndoorController_r2018_B.middle_x[0], 0, 10U * sizeof(real_T));
    memset(&RflysimIndoorController_r2018_B.middle_y[0], 0, 10U * sizeof(real_T));
    switch (1 + j) {
     case 1:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_h.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_h.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_h.Units_SL_Info.CurrentLength;
      break;

     case 2:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_j.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_j.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_j.Units_SL_Info.CurrentLength;
      break;

     case 3:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_mz.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_mz.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_mz.Units_SL_Info.CurrentLength;
      break;

     case 4:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_f.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_f.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_f.Units_SL_Info.CurrentLength;
      break;

     case 5:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_o.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_o.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_o.Units_SL_Info.CurrentLength;
      break;

     case 6:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_i.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_i.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_i.Units_SL_Info.CurrentLength;
      break;

     case 7:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_b.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_b.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_b.Units_SL_Info.CurrentLength;
      break;

     case 8:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_p.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_p.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_p.Units_SL_Info.CurrentLength;
      break;

     case 9:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new.Units_SL_Info.CurrentLength;
      break;

     case 10:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_m.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_m.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_m.Units_SL_Info.CurrentLength;
      break;

     case 11:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_pf.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_pf.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_pf.Units_SL_Info.CurrentLength;
      break;

     default:
      RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data =
        RflysimIndoorController_r2018_B.msg_new_m2.Pipetype.Data;
      tunnel_msg_Units = &RflysimIndoorController_r2018_B.msg_new_m2.Units[0];
      RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren =
        RflysimIndoorController_r2018_B.msg_new_m2.Units_SL_Info.CurrentLength;
      break;
    }

    for (rtb_tri2 = 0; rtb_tri2 < (int32_T)
         RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
         rtb_tri2++) {
      RflysimIndoorController_r2018_B.k = 1U + rtb_tri2;
      rtb_tri3 = (int32_T)RflysimIndoorController_r2018_B.k - 1;
      RflysimIndoorController_r2018_B.left_x[rtb_tri3] =
        tunnel_msg_Units[rtb_tri3].Left.X;
      RflysimIndoorController_r2018_B.left_y[rtb_tri3] = tunnel_msg_Units
        [(int32_T)RflysimIndoorController_r2018_B.k - 1].Left.Y;
      RflysimIndoorController_r2018_B.right_x[rtb_tri3] = tunnel_msg_Units
        [(int32_T)RflysimIndoorController_r2018_B.k - 1].Right.X;
      RflysimIndoorController_r2018_B.right_y[rtb_tri3] = tunnel_msg_Units
        [(int32_T)RflysimIndoorController_r2018_B.k - 1].Right.Y;
      RflysimIndoorController_r2018_B.middle_x[rtb_tri3] = tunnel_msg_Units
        [(int32_T)RflysimIndoorController_r2018_B.k - 1].Middle.X;
      RflysimIndoorController_r2018_B.middle_y[rtb_tri3] = tunnel_msg_Units
        [(int32_T)RflysimIndoorController_r2018_B.k - 1].Middle.Y;
    }

    if (1U > RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren) {
      rtb_tri2 = 0;
      rtb_tri3 = 0;
      rtb_tri4 = 0;
    } else {
      rtb_tri2 = (int32_T)
        RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
      rtb_tri3 = (int32_T)
        RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
      rtb_tri4 = (int32_T)
        RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
    }

    memset(&RflysimIndoorController_r2018_B.p_radius[0], 0, 1000U * sizeof
           (real_T));
    memset(&RflysimIndoorController_r2018_B.p_left_encircle[0], 0, 1000U *
           sizeof(real_T));
    memset(&RflysimIndoorController_r2018_B.p_right_encircle[0], 0, 1000U *
           sizeof(real_T));
    if (RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data == 10U) {
      if (rtb_tri4 < 1) {
        RflysimIndoorController_r2018_B.b_k_size[0] = 1;
        RflysimIndoorController_r2018_B.b_k_size[1] = 0;
      } else {
        RflysimIndoorController_r2018_B.b_k_size[0] = 1;
        RflysimIndoorController_r2018_B.b_k_size[1] = rtb_tri4;
        RflysimIndoorController_r2018_B.tri5 = rtb_tri4 - 1;
        for (RflysimIndoorController_r2018_B.tri6 = 0;
             RflysimIndoorController_r2018_B.tri6 <=
             RflysimIndoorController_r2018_B.tri5;
             RflysimIndoorController_r2018_B.tri6++) {
          RflysimIndoorController_r2018_B.b_k_data[RflysimIndoorController_r2018_B.tri6]
            = 1.0 + (real_T)RflysimIndoorController_r2018_B.tri6;
        }
      }

      RflysimIndoorControlle_linspace((real_T)rtb_tri4,
        RflysimIndoorController_r2018_B.ki);
      if (0 <= rtb_tri4 - 1) {
        memcpy(&RflysimIndoorController_r2018_B.middle_x_data[0],
               &RflysimIndoorController_r2018_B.middle_x[0], rtb_tri4 * sizeof
               (real_T));
      }

      RflysimIndoorController_interp1(RflysimIndoorController_r2018_B.b_k_data,
        RflysimIndoorController_r2018_B.b_k_size,
        RflysimIndoorController_r2018_B.middle_x_data, &rtb_tri4,
        RflysimIndoorController_r2018_B.ki, RflysimIndoorController_r2018_B.dv0);
      memcpy(&RflysimIndoorController_r2018_B.p_radius[0],
             &RflysimIndoorController_r2018_B.dv0[0], 500U * sizeof(real_T));
      if (1U > RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren)
      {
        RflysimIndoorController_r2018_B.tri5 = 0;
      } else {
        RflysimIndoorController_r2018_B.tri5 = (int32_T)
          RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
      }

      rtb_tri4 = RflysimIndoorController_r2018_B.tri5;
      if (0 <= RflysimIndoorController_r2018_B.tri5 - 1) {
        memcpy(&RflysimIndoorController_r2018_B.middle_x[0],
               &RflysimIndoorController_r2018_B.middle_y[0],
               RflysimIndoorController_r2018_B.tri5 * sizeof(real_T));
      }

      RflysimIndoorController_interp1(RflysimIndoorController_r2018_B.b_k_data,
        RflysimIndoorController_r2018_B.b_k_size,
        RflysimIndoorController_r2018_B.middle_x, &rtb_tri4,
        RflysimIndoorController_r2018_B.ki, RflysimIndoorController_r2018_B.dv0);
      memcpy(&RflysimIndoorController_r2018_B.p_radius[500],
             &RflysimIndoorController_r2018_B.dv0[0], 500U * sizeof(real_T));
      if (rtb_tri2 < 1) {
        RflysimIndoorController_r2018_B.b_k_size[0] = 1;
        RflysimIndoorController_r2018_B.b_k_size[1] = 0;
      } else {
        RflysimIndoorController_r2018_B.b_k_size[0] = 1;
        RflysimIndoorController_r2018_B.b_k_size[1] = rtb_tri2;
        RflysimIndoorController_r2018_B.tri5 = rtb_tri2 - 1;
        for (RflysimIndoorController_r2018_B.tri6 = 0;
             RflysimIndoorController_r2018_B.tri6 <=
             RflysimIndoorController_r2018_B.tri5;
             RflysimIndoorController_r2018_B.tri6++) {
          RflysimIndoorController_r2018_B.b_k_data[RflysimIndoorController_r2018_B.tri6]
            = 1.0 + (real_T)RflysimIndoorController_r2018_B.tri6;
        }
      }

      RflysimIndoorControlle_linspace((real_T)rtb_tri2,
        RflysimIndoorController_r2018_B.ki);
      if (0 <= rtb_tri2 - 1) {
        memcpy(&RflysimIndoorController_r2018_B.middle_y[0],
               &RflysimIndoorController_r2018_B.left_x[0], rtb_tri2 * sizeof
               (real_T));
      }

      RflysimIndoorController_interp1(RflysimIndoorController_r2018_B.b_k_data,
        RflysimIndoorController_r2018_B.b_k_size,
        RflysimIndoorController_r2018_B.middle_y, &rtb_tri2,
        RflysimIndoorController_r2018_B.ki, RflysimIndoorController_r2018_B.dv0);
      memcpy(&RflysimIndoorController_r2018_B.p_left_encircle[0],
             &RflysimIndoorController_r2018_B.dv0[0], 500U * sizeof(real_T));
      if (1U > RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren)
      {
        RflysimIndoorController_r2018_B.tri5 = 0;
      } else {
        RflysimIndoorController_r2018_B.tri5 = (int32_T)
          RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
      }

      rtb_tri4 = RflysimIndoorController_r2018_B.tri5;
      if (0 <= RflysimIndoorController_r2018_B.tri5 - 1) {
        memcpy(&RflysimIndoorController_r2018_B.left_x[0],
               &RflysimIndoorController_r2018_B.left_y[0],
               RflysimIndoorController_r2018_B.tri5 * sizeof(real_T));
      }

      RflysimIndoorController_interp1(RflysimIndoorController_r2018_B.b_k_data,
        RflysimIndoorController_r2018_B.b_k_size,
        RflysimIndoorController_r2018_B.left_x, &rtb_tri4,
        RflysimIndoorController_r2018_B.ki, RflysimIndoorController_r2018_B.dv0);
      memcpy(&RflysimIndoorController_r2018_B.p_left_encircle[500],
             &RflysimIndoorController_r2018_B.dv0[0], 500U * sizeof(real_T));
      if (rtb_tri3 < 1) {
        RflysimIndoorController_r2018_B.b_k_size[0] = 1;
        RflysimIndoorController_r2018_B.b_k_size[1] = 0;
      } else {
        RflysimIndoorController_r2018_B.b_k_size[0] = 1;
        RflysimIndoorController_r2018_B.b_k_size[1] = rtb_tri3;
        RflysimIndoorController_r2018_B.tri5 = rtb_tri3 - 1;
        for (RflysimIndoorController_r2018_B.tri6 = 0;
             RflysimIndoorController_r2018_B.tri6 <=
             RflysimIndoorController_r2018_B.tri5;
             RflysimIndoorController_r2018_B.tri6++) {
          RflysimIndoorController_r2018_B.b_k_data[RflysimIndoorController_r2018_B.tri6]
            = 1.0 + (real_T)RflysimIndoorController_r2018_B.tri6;
        }
      }

      RflysimIndoorControlle_linspace((real_T)rtb_tri3,
        RflysimIndoorController_r2018_B.ki);
      if (0 <= rtb_tri3 - 1) {
        memcpy(&RflysimIndoorController_r2018_B.left_y[0],
               &RflysimIndoorController_r2018_B.right_x[0], rtb_tri3 * sizeof
               (real_T));
      }

      RflysimIndoorController_interp1(RflysimIndoorController_r2018_B.b_k_data,
        RflysimIndoorController_r2018_B.b_k_size,
        RflysimIndoorController_r2018_B.left_y, &rtb_tri3,
        RflysimIndoorController_r2018_B.ki, RflysimIndoorController_r2018_B.dv0);
      memcpy(&RflysimIndoorController_r2018_B.p_right_encircle[0],
             &RflysimIndoorController_r2018_B.dv0[0], 500U * sizeof(real_T));
      if (1U > RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren)
      {
        RflysimIndoorController_r2018_B.tri5 = 0;
      } else {
        RflysimIndoorController_r2018_B.tri5 = (int32_T)
          RflysimIndoorController_r2018_B.tunnel_msg_Units_SL_Info_Curren;
      }

      rtb_tri4 = RflysimIndoorController_r2018_B.tri5;
      if (0 <= RflysimIndoorController_r2018_B.tri5 - 1) {
        memcpy(&RflysimIndoorController_r2018_B.right_x[0],
               &RflysimIndoorController_r2018_B.right_y[0],
               RflysimIndoorController_r2018_B.tri5 * sizeof(real_T));
      }

      RflysimIndoorController_interp1(RflysimIndoorController_r2018_B.b_k_data,
        RflysimIndoorController_r2018_B.b_k_size,
        RflysimIndoorController_r2018_B.right_x, &rtb_tri4,
        RflysimIndoorController_r2018_B.ki, RflysimIndoorController_r2018_B.dv0);
      memcpy(&RflysimIndoorController_r2018_B.p_right_encircle[500],
             &RflysimIndoorController_r2018_B.dv0[0], 500U * sizeof(real_T));
    }

    RflysimIndoorCont_Encirclement4(RflysimIndoorController_r2018_B.p_radius,
      RflysimIndoorController_r2018_B.p_left_encircle,
      RflysimIndoorController_r2018_B.p_right_encircle,
      RflysimIndoorController_r2018_P.p_target,
      RflysimIndoorController_r2018_B.Pcur, RflysimIndoorController_r2018_B.Vcur,
      RflysimIndoorController_r2018_B.TmpSignalConversionAtSFun_p,
      RflysimIndoorController_r2018_P.l_v, RflysimIndoorController_r2018_P.v_max,
      RflysimIndoorController_r2018_P.rs, RflysimIndoorController_r2018_P.ra,
      RflysimIndoorController_r2018_P.rt_s, RflysimIndoorController_r2018_P.rt_a,
      1.0 + (real_T)j, RflysimIndoorController_r2018_B.u_encircle,
      &RflysimIndoorController_r2018_B.ux_encircle,
      &RflysimIndoorController_r2018_B.uy_encircle,
      &RflysimIndoorController_r2018_B.uyaw_encircle,
      &RflysimIndoorController_r2018_B.leader_locate,
      &RflysimIndoorController_r2018_B.dist_encircle);
    if (RflysimIndoorController_r2018_B.rtb_TmpSignalConversionAtSFun_c[j] ==
        30.0) {
      RflysimIndoorController_r2018_B.states_new[j] = 30.0;
      if (RflysimIndoorController_r2018_B.tunnel_msg_Pipetype_Data == 10U) {
        RflysimIndoorController_r2018_B.ux[j] =
          RflysimIndoorController_r2018_B.ux_encircle;
        RflysimIndoorController_r2018_B.uy[j] =
          RflysimIndoorController_r2018_B.uy_encircle;
        RflysimIndoorController_r2018_B.uyaw[j] =
          RflysimIndoorController_r2018_B.uyaw_encircle;
        if (fabs(RflysimIndoorController_r2018_B.leader_locate - 500.0) < 5.0) {
          RflysimIndoorController_r2018_B.states_new[j] = 20.0;
        }
      }
    }
  }

  // MATLAB Function: '<Root>/MATLAB Function3' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function2'
  //   SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1'

  j = 0;
  rtb_tri2 = 0;
  rtb_tri3 = 0;
  rtb_tri4 = 0;
  RflysimIndoorController_r2018_B.tri5 = 0;
  RflysimIndoorController_r2018_B.tri6 = 0;
  RflysimIndoorController_r2018_B.tri7 = 0;
  RflysimIndoorController_r2018_B.tri8 = 0;
  RflysimIndoorController_r2018_B.tri9 = 0;
  RflysimIndoorController_r2018_B.tri10 = 0;
  RflysimIndoorController_r2018_B.tri11 = 0;
  RflysimIndoorController_r2018_B.tri12 = 0;
  if (RflysimIndoorController_r2018_B.In1_ks.Data == 30.0) {
    j = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_h3.Data == 30.0) {
    rtb_tri2 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_lp.Data == 30.0) {
    rtb_tri3 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_n0.Data == 30.0) {
    rtb_tri4 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_ox.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri5 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_dj.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri6 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_hr.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri7 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_fk.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri8 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_er.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri9 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_o.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri10 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_b1.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri11 = 1;
  }

  if (RflysimIndoorController_r2018_B.In1_a1.Data == 30.0) {
    RflysimIndoorController_r2018_B.tri12 = 1;
  }

  // Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
  //   EnablePort: '<S30>/Enable'

  if (j > 0) {
    // BusAssignment: '<S30>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_k.Data =
      RflysimIndoorController_r2018_B.states_new[0];

    // Outputs for Atomic SubSystem: '<S30>/Publish'
    // MATLABSystem: '<S142>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15746.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_k);

    // End of Outputs for SubSystem: '<S30>/Publish'

    // BusAssignment: '<S30>/Bus Assignment1' incorporates:
    //   Constant: '<S141>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_nc;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[0];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[0];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[0];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[0];

    // Outputs for Atomic SubSystem: '<S30>/Publish1'
    // MATLABSystem: '<S143>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15747.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S30>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem1' incorporates:
  //   EnablePort: '<S31>/Enable'

  if (rtb_tri2 > 0) {
    // BusAssignment: '<S31>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_c.Data =
      RflysimIndoorController_r2018_B.states_new[1];

    // Outputs for Atomic SubSystem: '<S31>/Publish'
    // MATLABSystem: '<S146>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15870.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_c);

    // End of Outputs for SubSystem: '<S31>/Publish'

    // BusAssignment: '<S31>/Bus Assignment1' incorporates:
    //   Constant: '<S145>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_ak;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[1];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[1];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[1];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[1];

    // Outputs for Atomic SubSystem: '<S31>/Publish1'
    // MATLABSystem: '<S147>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15871.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S31>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem1'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem10' incorporates:
  //   EnablePort: '<S32>/Enable'

  if (RflysimIndoorController_r2018_B.tri11 > 0) {
    // BusAssignment: '<S32>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_et.Data =
      RflysimIndoorController_r2018_B.states_new[10];

    // Outputs for Atomic SubSystem: '<S32>/Publish'
    // MATLABSystem: '<S150>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16174.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_et);

    // End of Outputs for SubSystem: '<S32>/Publish'

    // BusAssignment: '<S32>/Bus Assignment1' incorporates:
    //   Constant: '<S149>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_lo;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[10];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[10];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[10];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[10];

    // Outputs for Atomic SubSystem: '<S32>/Publish1'
    // MATLABSystem: '<S151>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16175.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S32>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem10'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem11' incorporates:
  //   EnablePort: '<S33>/Enable'

  if (RflysimIndoorController_r2018_B.tri12 > 0) {
    // BusAssignment: '<S33>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_a4.Data =
      RflysimIndoorController_r2018_B.states_new[11];

    // Outputs for Atomic SubSystem: '<S33>/Publish'
    // MATLABSystem: '<S154>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16187.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_a4);

    // End of Outputs for SubSystem: '<S33>/Publish'

    // BusAssignment: '<S33>/Bus Assignment1' incorporates:
    //   Constant: '<S153>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_pp;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[11];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[11];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[11];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[11];

    // Outputs for Atomic SubSystem: '<S33>/Publish1'
    // MATLABSystem: '<S155>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16188.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S33>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem11'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem2' incorporates:
  //   EnablePort: '<S34>/Enable'

  if (rtb_tri3 > 0) {
    // BusAssignment: '<S34>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_e.Data =
      RflysimIndoorController_r2018_B.states_new[2];

    // Outputs for Atomic SubSystem: '<S34>/Publish'
    // MATLABSystem: '<S158>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15883.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_e);

    // End of Outputs for SubSystem: '<S34>/Publish'

    // BusAssignment: '<S34>/Bus Assignment1' incorporates:
    //   Constant: '<S157>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_gd;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[2];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[2];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[2];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[2];

    // Outputs for Atomic SubSystem: '<S34>/Publish1'
    // MATLABSystem: '<S159>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15884.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S34>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem2'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem3' incorporates:
  //   EnablePort: '<S35>/Enable'

  if (rtb_tri4 > 0) {
    // BusAssignment: '<S35>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_b.Data =
      RflysimIndoorController_r2018_B.states_new[3];

    // Outputs for Atomic SubSystem: '<S35>/Publish'
    // MATLABSystem: '<S162>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15973.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_b);

    // End of Outputs for SubSystem: '<S35>/Publish'

    // BusAssignment: '<S35>/Bus Assignment1' incorporates:
    //   Constant: '<S161>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_ej;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[3];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[3];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[3];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[3];

    // Outputs for Atomic SubSystem: '<S35>/Publish1'
    // MATLABSystem: '<S163>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15974.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S35>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem3'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem4' incorporates:
  //   EnablePort: '<S36>/Enable'

  if (RflysimIndoorController_r2018_B.tri5 > 0) {
    // BusAssignment: '<S36>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_j.Data =
      RflysimIndoorController_r2018_B.states_new[4];

    // Outputs for Atomic SubSystem: '<S36>/Publish'
    // MATLABSystem: '<S166>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15986.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_j);

    // End of Outputs for SubSystem: '<S36>/Publish'

    // BusAssignment: '<S36>/Bus Assignment1' incorporates:
    //   Constant: '<S165>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_kd;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[4];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[4];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[4];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[4];

    // Outputs for Atomic SubSystem: '<S36>/Publish1'
    // MATLABSystem: '<S167>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15987.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S36>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem4'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem5' incorporates:
  //   EnablePort: '<S37>/Enable'

  if (RflysimIndoorController_r2018_B.tri6 > 0) {
    // BusAssignment: '<S37>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_i.Data =
      RflysimIndoorController_r2018_B.states_new[5];

    // Outputs for Atomic SubSystem: '<S37>/Publish'
    // MATLABSystem: '<S170>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_15999.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_i);

    // End of Outputs for SubSystem: '<S37>/Publish'

    // BusAssignment: '<S37>/Bus Assignment1' incorporates:
    //   Constant: '<S169>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_lj;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[5];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[5];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[5];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[5];

    // Outputs for Atomic SubSystem: '<S37>/Publish1'
    // MATLABSystem: '<S171>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16000.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S37>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem5'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem6' incorporates:
  //   EnablePort: '<S38>/Enable'

  if (RflysimIndoorController_r2018_B.tri7 > 0) {
    // BusAssignment: '<S38>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_a.Data =
      RflysimIndoorController_r2018_B.states_new[6];

    // Outputs for Atomic SubSystem: '<S38>/Publish'
    // MATLABSystem: '<S174>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16122.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_a);

    // End of Outputs for SubSystem: '<S38>/Publish'

    // BusAssignment: '<S38>/Bus Assignment1' incorporates:
    //   Constant: '<S173>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_lp;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[6];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[6];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[6];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[6];

    // Outputs for Atomic SubSystem: '<S38>/Publish1'
    // MATLABSystem: '<S175>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16123.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S38>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem6'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem7' incorporates:
  //   EnablePort: '<S39>/Enable'

  if (RflysimIndoorController_r2018_B.tri8 > 0) {
    // BusAssignment: '<S39>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_o.Data =
      RflysimIndoorController_r2018_B.states_new[7];

    // Outputs for Atomic SubSystem: '<S39>/Publish'
    // MATLABSystem: '<S178>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16135.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_o);

    // End of Outputs for SubSystem: '<S39>/Publish'

    // BusAssignment: '<S39>/Bus Assignment1' incorporates:
    //   Constant: '<S177>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_fx;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[7];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[7];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[7];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[7];

    // Outputs for Atomic SubSystem: '<S39>/Publish1'
    // MATLABSystem: '<S179>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16136.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S39>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem7'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem8' incorporates:
  //   EnablePort: '<S40>/Enable'

  if (RflysimIndoorController_r2018_B.tri9 > 0) {
    // BusAssignment: '<S40>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment_g.Data =
      RflysimIndoorController_r2018_B.states_new[8];

    // Outputs for Atomic SubSystem: '<S40>/Publish'
    // MATLABSystem: '<S182>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16148.publish
      (&RflysimIndoorController_r2018_B.BusAssignment_g);

    // End of Outputs for SubSystem: '<S40>/Publish'

    // BusAssignment: '<S40>/Bus Assignment1' incorporates:
    //   Constant: '<S181>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_f3;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[8];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[8];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[8];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[8];

    // Outputs for Atomic SubSystem: '<S40>/Publish1'
    // MATLABSystem: '<S183>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16149.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S40>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem8'

  // Outputs for Enabled SubSystem: '<Root>/Subsystem9' incorporates:
  //   EnablePort: '<S41>/Enable'

  if (RflysimIndoorController_r2018_B.tri10 > 0) {
    // BusAssignment: '<S41>/Bus Assignment'
    RflysimIndoorController_r2018_B.BusAssignment.Data =
      RflysimIndoorController_r2018_B.states_new[9];

    // Outputs for Atomic SubSystem: '<S41>/Publish'
    // MATLABSystem: '<S186>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16161.publish
      (&RflysimIndoorController_r2018_B.BusAssignment);

    // End of Outputs for SubSystem: '<S41>/Publish'

    // BusAssignment: '<S41>/Bus Assignment1' incorporates:
    //   Constant: '<S185>/Constant'

    RflysimIndoorController_r2018_B.BusAssignment1_j =
      RflysimIndoorController_r2018_P.Constant_Value_be;
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.X =
      RflysimIndoorController_r2018_B.ux[9];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Y =
      RflysimIndoorController_r2018_B.uy[9];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Linear.Z =
      RflysimIndoorController_r2018_B.uz[9];
    RflysimIndoorController_r2018_B.BusAssignment1_j.Twist.Angular.Z =
      RflysimIndoorController_r2018_B.uyaw[9];

    // Outputs for Atomic SubSystem: '<S41>/Publish1'
    // MATLABSystem: '<S187>/SinkBlock'
    Pub_RflysimIndoorController_r2018b_N12_V9_16162.publish
      (&RflysimIndoorController_r2018_B.BusAssignment1_j);

    // End of Outputs for SubSystem: '<S41>/Publish1'
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem9'

  // S-Function (saeroclockpacer): '<Root>/Simulation Pace'
  //
  //  The Clock Pacer generates no code, it is only active in
  //  interpreted simulation.


  // Update for Memory: '<Root>/Memory4'
  RflysimIndoorController_r201_DW.Memory4_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_h;

  // Update for Memory: '<Root>/Memory5'
  RflysimIndoorController_r201_DW.Memory5_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_j;

  // Update for Memory: '<Root>/Memory6'
  RflysimIndoorController_r201_DW.Memory6_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_mz;

  // Update for Memory: '<Root>/Memory1'
  RflysimIndoorController_r201_DW.Memory1_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_f;

  // Update for Memory: '<Root>/Memory2'
  RflysimIndoorController_r201_DW.Memory2_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_o;

  // Update for Memory: '<Root>/Memory3'
  RflysimIndoorController_r201_DW.Memory3_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_i;

  // Update for Memory: '<Root>/Memory10'
  RflysimIndoorController_r201_DW.Memory10_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_b;

  // Update for Memory: '<Root>/Memory11'
  RflysimIndoorController_r201_DW.Memory11_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_p;

  // Update for Memory: '<Root>/Memory12'
  RflysimIndoorController_r201_DW.Memory12_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new;

  // Update for Memory: '<Root>/Memory7'
  RflysimIndoorController_r201_DW.Memory7_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_m;

  // Update for Memory: '<Root>/Memory8'
  RflysimIndoorController_r201_DW.Memory8_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_pf;

  // Update for Memory: '<Root>/Memory9'
  RflysimIndoorController_r201_DW.Memory9_PreviousInput =
    RflysimIndoorController_r2018_B.msg_new_m2;
}

// Model initialize function
void RflysimIndoorController_r2018b_N12_V9_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    static const char_T tmp[42] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1', '0',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_0[15] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '0', '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_1[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '9',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_2[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '9',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_3[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '8',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_4[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '8',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_5[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '7',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_6[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '7',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_7[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '6',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_8[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '6',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_9[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '5',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_a[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '5',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_b[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '4',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_c[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '4',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_d[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '3',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_e[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '3',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_f[42] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '2', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i',
      'n', 't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_g[15] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '2', '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_h[42] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '1', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i',
      'n', 't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_i[15] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '1', '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_j[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '2',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_k[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '2',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_l[41] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 's', 'e', 't', 'p', 'o', 'i', 'n',
      't', '_', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '/', 'c', 'm', 'd', '_',
      'v', 'e', 'l' };

    static const char_T tmp_m[14] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '/', 's', 't', 'a', 't', 'e' };

    static const char_T tmp_n[46] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '2', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_',
      'p', 'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i',
      't', 'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_o[46] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '1', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_',
      'p', 'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i',
      't', 'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_p[46] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '0', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_',
      'p', 'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i',
      't', 'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_q[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '9',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_r[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '8',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_s[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '7',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_t[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '6',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_u[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '5',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_v[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '4',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_w[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '3',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_x[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '2',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_y[45] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'v', 'e', 'l', 'o', 'c', 'i', 't',
      'y', '_', 'l', 'o', 'c', 'a', 'l' };

    static const char_T tmp_z[40] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '2', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_',
      'p', 'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c',
      'o', 'r' };

    static const char_T tmp_10[40] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '1', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_',
      'p', 'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c',
      'o', 'r' };

    static const char_T tmp_11[40] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '0', '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_',
      'p', 'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c',
      'o', 'r' };

    static const char_T tmp_12[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '9',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_13[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '8',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_14[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '7',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_15[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '6',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_16[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '5',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_17[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '4',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_18[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '3',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_19[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '2',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_1a[39] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '/', 'm', 'a', 'v', 'r', 'o', 's', '/', 'l', 'o', 'c', 'a', 'l', '_', 'p',
      'o', 's', 'i', 't', 'i', 'o', 'n', '/', 'p', 'o', 's', 'e', '_', 'c', 'o',
      'r' };

    static const char_T tmp_1b[23] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '2', '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o'
    };

    static const char_T tmp_1c[23] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '1', '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o'
    };

    static const char_T tmp_1d[23] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '0', '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o'
    };

    static const char_T tmp_1e[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '9',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1f[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '8',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1g[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '7',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1h[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '6',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1i[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '5',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1j[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '4',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1k[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '3',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1l[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '2',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    static const char_T tmp_1m[22] = { '/', 'd', 'r', 'o', 'n', 'e', '_', '1',
      '/', 'd', 'e', 'c', 'i', 's', 'i', 'o', 'n', '_', 'i', 'n', 'f', 'o' };

    char_T tmp_1n[23];
    char_T tmp_1o[16];
    char_T tmp_1p[15];
    int32_T i;

    // Start for Atomic SubSystem: '<S17>/Subscribe'
    // Start for MATLABSystem: '<S104>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_n0.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_n0.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_n0.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_n0.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_n0.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_m[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15717.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_n0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S104>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe'

    // Start for Atomic SubSystem: '<S17>/Subscribe4'
    // Start for MATLABSystem: '<S110>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_jh.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_jh.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_jh.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_jh.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_jh.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_k[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15833.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_jh.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S110>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe4'

    // Start for Atomic SubSystem: '<S17>/Subscribe8'
    // Start for MATLABSystem: '<S114>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_n.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_n.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_n.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_n.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_e[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15851.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S114>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe8'

    // Start for Atomic SubSystem: '<S17>/Subscribe1'
    // Start for MATLABSystem: '<S105>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_lv.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_lv.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_lv.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_lv.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_lv.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_c[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15920.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_lv.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S105>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe1'

    // Start for Atomic SubSystem: '<S17>/Subscribe2'
    // Start for MATLABSystem: '<S108>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_pf.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_pf.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_pf.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_pf.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_pf.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_a[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15921.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_pf.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S108>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe2'

    // Start for Atomic SubSystem: '<S17>/Subscribe3'
    // Start for MATLABSystem: '<S109>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_mk.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_mk.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_mk.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_mk.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_mk.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_8[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15922.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_mk.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S109>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe3'

    // Start for Atomic SubSystem: '<S17>/Subscribe5'
    // Start for MATLABSystem: '<S111>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_f.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_f.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_f.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_f.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_6[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16033.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S111>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe5'

    // Start for Atomic SubSystem: '<S17>/Subscribe10'
    // Start for MATLABSystem: '<S106>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_o.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_o.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_o.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_o.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_4[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16037.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S106>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe10'

    // Start for Atomic SubSystem: '<S17>/Subscribe11'
    // Start for MATLABSystem: '<S107>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_iw.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_iw.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_iw.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_iw.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_iw.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_2[i];
    }

    tmp_1p[14] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16038.createSubscriber(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_iw.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S107>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe11'

    // Start for Atomic SubSystem: '<S17>/Subscribe6'
    // Start for MATLABSystem: '<S112>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_k4n.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_k4n.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_k4n.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_k4n.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_k4n.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_1o[i] = tmp_0[i];
    }

    tmp_1o[15] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16034.createSubscriber(tmp_1o, 1);
    RflysimIndoorController_r201_DW.obj_k4n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S112>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe6'

    // Start for Atomic SubSystem: '<S17>/Subscribe7'
    // Start for MATLABSystem: '<S113>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_k4.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_k4.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_k4.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_k4.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_k4.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_1o[i] = tmp_i[i];
    }

    tmp_1o[15] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16035.createSubscriber(tmp_1o, 1);
    RflysimIndoorController_r201_DW.obj_k4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S113>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe7'

    // Start for Atomic SubSystem: '<S17>/Subscribe9'
    // Start for MATLABSystem: '<S115>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_m.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_m.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_m.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_m.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_1o[i] = tmp_g[i];
    }

    tmp_1o[15] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16036.createSubscriber(tmp_1o, 1);
    RflysimIndoorController_r201_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S115>/SourceBlock'
    // End of Start for SubSystem: '<S17>/Subscribe9'

    // Start for Atomic SubSystem: '<Root>/Subscribe3'
    // Start for MATLABSystem: '<S23>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_b.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_b.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_b.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_b.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1m[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15807.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S23>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe3'

    // Start for Atomic SubSystem: '<Root>/Subscribe7'
    // Start for MATLABSystem: '<S27>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_k.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_k.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_k.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_k.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1l[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15836.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S27>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe7'

    // Start for Atomic SubSystem: '<Root>/Subscribe11'
    // Start for MATLABSystem: '<S20>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_j.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_j.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_j.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_j.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1k[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15854.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S20>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe11'

    // Start for Atomic SubSystem: '<Root>/Subscribe2'
    // Start for MATLABSystem: '<S22>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_a.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_a.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_a.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_a.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1j[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15933.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S22>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe2'

    // Start for Atomic SubSystem: '<Root>/Subscribe4'
    // Start for MATLABSystem: '<S24>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_i.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_i.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_i.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_i.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1i[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15934.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S24>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe4'

    // Start for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S18>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_aj.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_aj.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_aj.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_aj.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_aj.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1h[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15932.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_aj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S18>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe1'

    // Start for Atomic SubSystem: '<Root>/Subscribe9'
    // Start for MATLABSystem: '<S29>/SourceBlock'
    RflysimIndoorController_r201_DW.obj.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1g[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16018.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S29>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe9'

    // Start for Atomic SubSystem: '<Root>/Subscribe12'
    // Start for MATLABSystem: '<S21>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_bb.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bb.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bb.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bb.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bb.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1f[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16020.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_bb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe12'

    // Start for Atomic SubSystem: '<Root>/Subscribe6'
    // Start for MATLABSystem: '<S26>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_ps.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_ps.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_ps.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_ps.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_ps.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_1n[i] = tmp_1e[i];
    }

    tmp_1n[22] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16016.createSubscriber(tmp_1n, 1);
    RflysimIndoorController_r201_DW.obj_ps.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe6'

    // Start for Atomic SubSystem: '<Root>/Subscribe8'
    // Start for MATLABSystem: '<S28>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_p.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_p.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_p.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_p.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      RflysimIndoorController_r2018_B.cv6[i] = tmp_1d[i];
    }

    RflysimIndoorController_r2018_B.cv6[23] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16017.createSubscriber
      (RflysimIndoorController_r2018_B.cv6, 1);
    RflysimIndoorController_r201_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S28>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe8'

    // Start for Atomic SubSystem: '<Root>/Subscribe10'
    // Start for MATLABSystem: '<S19>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_h.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_h.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_h.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_h.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      RflysimIndoorController_r2018_B.cv6[i] = tmp_1c[i];
    }

    RflysimIndoorController_r2018_B.cv6[23] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16019.createSubscriber
      (RflysimIndoorController_r2018_B.cv6, 1);
    RflysimIndoorController_r201_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe10'

    // Start for Atomic SubSystem: '<Root>/Subscribe5'
    // Start for MATLABSystem: '<S25>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_l.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_l.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_l.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_l.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      RflysimIndoorController_r2018_B.cv6[i] = tmp_1b[i];
    }

    RflysimIndoorController_r2018_B.cv6[23] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16015.createSubscriber
      (RflysimIndoorController_r2018_B.cv6, 1);
    RflysimIndoorController_r201_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S25>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe5'

    // Start for Atomic SubSystem: '<S16>/Subscribe1'
    // Start for MATLABSystem: '<S56>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_dy.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_dy.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_dy.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_dy.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_dy.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_1a[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15722.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_dy.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S56>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe1'

    // Start for Atomic SubSystem: '<S16>/Subscribe5'
    // Start for MATLABSystem: '<S75>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_g.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_g.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_g.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_g.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_19[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15834.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S75>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe5'

    // Start for Atomic SubSystem: '<S16>/Subscribe9'
    // Start for MATLABSystem: '<S79>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_hv.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_hv.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_hv.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_hv.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_hv.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_18[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15852.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_hv.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S79>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe9'

    // Start for Atomic SubSystem: '<S16>/Subscribe3'
    // Start for MATLABSystem: '<S73>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_bs.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bs.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bs.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bs.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bs.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_17[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15947.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_bs.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S73>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe3'

    // Start for Atomic SubSystem: '<S16>/Subscribe4'
    // Start for MATLABSystem: '<S74>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_ma.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_ma.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_ma.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_ma.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_ma.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_16[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15948.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_ma.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S74>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe4'

    // Start for Atomic SubSystem: '<S16>/Subscribe7'
    // Start for MATLABSystem: '<S77>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_nq.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_nq.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_nq.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_nq.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_nq.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_15[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15949.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_nq.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S77>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe7'

    // Start for Atomic SubSystem: '<S16>/Subscribe13'
    // Start for MATLABSystem: '<S60>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_eu.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_eu.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_eu.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_eu.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_eu.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_14[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16063.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_eu.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S60>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe13'

    // Start for Atomic SubSystem: '<S16>/Subscribe16'
    // Start for MATLABSystem: '<S63>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_e.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_e.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_e.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_e.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_13[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16066.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S63>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe16'

    // Start for Atomic SubSystem: '<S16>/Subscribe18'
    // Start for MATLABSystem: '<S65>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_d.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_d.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_d.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_d.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      RflysimIndoorController_r2018_B.cv5[i] = tmp_12[i];
    }

    RflysimIndoorController_r2018_B.cv5[39] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16068.createSubscriber
      (RflysimIndoorController_r2018_B.cv5, 1);
    RflysimIndoorController_r201_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S65>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe18'

    // Start for Atomic SubSystem: '<S16>/Subscribe14'
    // Start for MATLABSystem: '<S61>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_jr.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_jr.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_jr.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_jr.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_jr.isInitialized = 1;
    for (i = 0; i < 40; i++) {
      RflysimIndoorController_r2018_B.cv4[i] = tmp_11[i];
    }

    RflysimIndoorController_r2018_B.cv4[40] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16064.createSubscriber
      (RflysimIndoorController_r2018_B.cv4, 1);
    RflysimIndoorController_r201_DW.obj_jr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S61>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe14'

    // Start for Atomic SubSystem: '<S16>/Subscribe15'
    // Start for MATLABSystem: '<S62>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_fr.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_fr.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_fr.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_fr.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_fr.isInitialized = 1;
    for (i = 0; i < 40; i++) {
      RflysimIndoorController_r2018_B.cv4[i] = tmp_10[i];
    }

    RflysimIndoorController_r2018_B.cv4[40] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16065.createSubscriber
      (RflysimIndoorController_r2018_B.cv4, 1);
    RflysimIndoorController_r201_DW.obj_fr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S62>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe15'

    // Start for Atomic SubSystem: '<S16>/Subscribe17'
    // Start for MATLABSystem: '<S64>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_mh.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_mh.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_mh.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_mh.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_mh.isInitialized = 1;
    for (i = 0; i < 40; i++) {
      RflysimIndoorController_r2018_B.cv4[i] = tmp_z[i];
    }

    RflysimIndoorController_r2018_B.cv4[40] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16067.createSubscriber
      (RflysimIndoorController_r2018_B.cv4, 1);
    RflysimIndoorController_r201_DW.obj_mh.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S64>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe17'

    // Start for Atomic SubSystem: '<S16>/Subscribe2'
    // Start for MATLABSystem: '<S67>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_pa.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_pa.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_pa.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_pa.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_pa.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_y[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15777.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_pa.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S67>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe2'

    // Start for Atomic SubSystem: '<S16>/Subscribe6'
    // Start for MATLABSystem: '<S76>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_bbp.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bbp.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bbp.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bbp.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bbp.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_x[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15835.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_bbp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S76>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe6'

    // Start for Atomic SubSystem: '<S16>/Subscribe10'
    // Start for MATLABSystem: '<S57>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_ot.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_ot.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_ot.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_ot.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_ot.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_w[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15853.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_ot.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S57>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe10'

    // Start for Atomic SubSystem: '<S16>/Subscribe11'
    // Start for MATLABSystem: '<S58>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_c.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_c.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_c.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_c.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_v[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15957.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S58>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe11'

    // Start for Atomic SubSystem: '<S16>/Subscribe12'
    // Start for MATLABSystem: '<S59>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_fe.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_fe.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_fe.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_fe.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_fe.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_u[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15958.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_fe.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S59>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe12'

    // Start for Atomic SubSystem: '<S16>/Subscribe8'
    // Start for MATLABSystem: '<S78>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_bm.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bm.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bm.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bm.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bm.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_t[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_15956.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_bm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S78>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe8'

    // Start for Atomic SubSystem: '<S16>/Subscribe22'
    // Start for MATLABSystem: '<S70>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_al.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_al.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_al.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_al.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_al.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_s[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16102.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_al.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S70>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe22'

    // Start for Atomic SubSystem: '<S16>/Subscribe23'
    // Start for MATLABSystem: '<S71>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_bp.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bp.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bp.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bp.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bp.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_r[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16103.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_bp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S71>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe23'

    // Start for Atomic SubSystem: '<S16>/Subscribe19'
    // Start for MATLABSystem: '<S66>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_hm.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_hm.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_hm.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_hm.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_hm.isInitialized = 1;
    for (i = 0; i < 45; i++) {
      RflysimIndoorController_r2018_B.cv1[i] = tmp_q[i];
    }

    RflysimIndoorController_r2018_B.cv1[45] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16099.createSubscriber
      (RflysimIndoorController_r2018_B.cv1, 1);
    RflysimIndoorController_r201_DW.obj_hm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S66>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe19'

    // Start for Atomic SubSystem: '<S16>/Subscribe20'
    // Start for MATLABSystem: '<S68>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_av.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_av.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_av.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_av.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_av.isInitialized = 1;
    for (i = 0; i < 46; i++) {
      RflysimIndoorController_r2018_B.cv0[i] = tmp_p[i];
    }

    RflysimIndoorController_r2018_B.cv0[46] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16100.createSubscriber
      (RflysimIndoorController_r2018_B.cv0, 1);
    RflysimIndoorController_r201_DW.obj_av.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S68>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe20'

    // Start for Atomic SubSystem: '<S16>/Subscribe21'
    // Start for MATLABSystem: '<S69>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_hc.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_hc.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_hc.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_hc.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_hc.isInitialized = 1;
    for (i = 0; i < 46; i++) {
      RflysimIndoorController_r2018_B.cv0[i] = tmp_o[i];
    }

    RflysimIndoorController_r2018_B.cv0[46] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16101.createSubscriber
      (RflysimIndoorController_r2018_B.cv0, 1);
    RflysimIndoorController_r201_DW.obj_hc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S69>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe21'

    // Start for Atomic SubSystem: '<S16>/Subscribe24'
    // Start for MATLABSystem: '<S72>/SourceBlock'
    RflysimIndoorController_r201_DW.obj_he.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_he.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_he.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_he.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_he.isInitialized = 1;
    for (i = 0; i < 46; i++) {
      RflysimIndoorController_r2018_B.cv0[i] = tmp_n[i];
    }

    RflysimIndoorController_r2018_B.cv0[46] = '\x00';
    Sub_RflysimIndoorController_r2018b_N12_V9_16104.createSubscriber
      (RflysimIndoorController_r2018_B.cv0, 1);
    RflysimIndoorController_r201_DW.obj_he.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S72>/SourceBlock'
    // End of Start for SubSystem: '<S16>/Subscribe24'

    // Start for Enabled SubSystem: '<Root>/Subsystem'
    // Start for Atomic SubSystem: '<S30>/Publish'
    // Start for MATLABSystem: '<S142>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_o3.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_o3.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_o3.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_o3.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_o3.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_m[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15746.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_o3.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S142>/SinkBlock'
    // End of Start for SubSystem: '<S30>/Publish'

    // Start for Atomic SubSystem: '<S30>/Publish1'
    // Start for MATLABSystem: '<S143>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_gn.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_gn.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_gn.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_gn.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_gn.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_l[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15747.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_gn.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S143>/SinkBlock'
    // End of Start for SubSystem: '<S30>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem'

    // Start for Enabled SubSystem: '<Root>/Subsystem1'
    // Start for Atomic SubSystem: '<S31>/Publish'
    // Start for MATLABSystem: '<S146>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_hd.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_hd.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_hd.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_hd.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_hd.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_k[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15870.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_hd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S146>/SinkBlock'
    // End of Start for SubSystem: '<S31>/Publish'

    // Start for Atomic SubSystem: '<S31>/Publish1'
    // Start for MATLABSystem: '<S147>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_cp.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_cp.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_cp.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_cp.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_cp.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_j[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15871.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_cp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S147>/SinkBlock'
    // End of Start for SubSystem: '<S31>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem1'

    // Start for Enabled SubSystem: '<Root>/Subsystem10'
    // Start for Atomic SubSystem: '<S32>/Publish'
    // Start for MATLABSystem: '<S150>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_j4t.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_j4t.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_j4t.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_j4t.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_j4t.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_1o[i] = tmp_i[i];
    }

    tmp_1o[15] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16174.createPublisher(tmp_1o, 1);
    RflysimIndoorController_r201_DW.obj_j4t.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S150>/SinkBlock'
    // End of Start for SubSystem: '<S32>/Publish'

    // Start for Atomic SubSystem: '<S32>/Publish1'
    // Start for MATLABSystem: '<S151>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_hx.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_hx.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_hx.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_hx.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_hx.isInitialized = 1;
    for (i = 0; i < 42; i++) {
      RflysimIndoorController_r2018_B.cv2[i] = tmp_h[i];
    }

    RflysimIndoorController_r2018_B.cv2[42] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16175.createPublisher
      (RflysimIndoorController_r2018_B.cv2, 1);
    RflysimIndoorController_r201_DW.obj_hx.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S151>/SinkBlock'
    // End of Start for SubSystem: '<S32>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem10'

    // Start for Enabled SubSystem: '<Root>/Subsystem11'
    // Start for Atomic SubSystem: '<S33>/Publish'
    // Start for MATLABSystem: '<S154>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_id.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_id.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_id.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_id.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_id.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_1o[i] = tmp_g[i];
    }

    tmp_1o[15] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16187.createPublisher(tmp_1o, 1);
    RflysimIndoorController_r201_DW.obj_id.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S154>/SinkBlock'
    // End of Start for SubSystem: '<S33>/Publish'

    // Start for Atomic SubSystem: '<S33>/Publish1'
    // Start for MATLABSystem: '<S155>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_l0.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_l0.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_l0.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_l0.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_l0.isInitialized = 1;
    for (i = 0; i < 42; i++) {
      RflysimIndoorController_r2018_B.cv2[i] = tmp_f[i];
    }

    RflysimIndoorController_r2018_B.cv2[42] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16188.createPublisher
      (RflysimIndoorController_r2018_B.cv2, 1);
    RflysimIndoorController_r201_DW.obj_l0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S155>/SinkBlock'
    // End of Start for SubSystem: '<S33>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem11'

    // Start for Enabled SubSystem: '<Root>/Subsystem2'
    // Start for Atomic SubSystem: '<S34>/Publish'
    // Start for MATLABSystem: '<S158>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_cm.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_cm.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_cm.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_cm.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_cm.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_e[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15883.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_cm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S158>/SinkBlock'
    // End of Start for SubSystem: '<S34>/Publish'

    // Start for Atomic SubSystem: '<S34>/Publish1'
    // Start for MATLABSystem: '<S159>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_j4.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_j4.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_j4.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_j4.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_j4.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_d[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15884.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_j4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S159>/SinkBlock'
    // End of Start for SubSystem: '<S34>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem2'

    // Start for Enabled SubSystem: '<Root>/Subsystem3'
    // Start for Atomic SubSystem: '<S35>/Publish'
    // Start for MATLABSystem: '<S162>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_bt.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bt.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bt.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bt.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bt.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_c[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15973.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_bt.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S162>/SinkBlock'
    // End of Start for SubSystem: '<S35>/Publish'

    // Start for Atomic SubSystem: '<S35>/Publish1'
    // Start for MATLABSystem: '<S163>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_kb.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_kb.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_kb.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_kb.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_kb.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_b[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15974.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_kb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S163>/SinkBlock'
    // End of Start for SubSystem: '<S35>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem3'

    // Start for Enabled SubSystem: '<Root>/Subsystem4'
    // Start for Atomic SubSystem: '<S36>/Publish'
    // Start for MATLABSystem: '<S166>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_fu.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_fu.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_fu.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_fu.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_fu.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_a[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15986.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_fu.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S166>/SinkBlock'
    // End of Start for SubSystem: '<S36>/Publish'

    // Start for Atomic SubSystem: '<S36>/Publish1'
    // Start for MATLABSystem: '<S167>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_bc.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_bc.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_bc.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_bc.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_bc.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_9[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15987.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_bc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S167>/SinkBlock'
    // End of Start for SubSystem: '<S36>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem4'

    // Start for Enabled SubSystem: '<Root>/Subsystem5'
    // Start for Atomic SubSystem: '<S37>/Publish'
    // Start for MATLABSystem: '<S170>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_k5.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_k5.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_k5.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_k5.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_k5.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_8[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_15999.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_k5.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S170>/SinkBlock'
    // End of Start for SubSystem: '<S37>/Publish'

    // Start for Atomic SubSystem: '<S37>/Publish1'
    // Start for MATLABSystem: '<S171>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_k0.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_k0.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_k0.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_k0.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_k0.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_7[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16000.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_k0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S171>/SinkBlock'
    // End of Start for SubSystem: '<S37>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem5'

    // Start for Enabled SubSystem: '<Root>/Subsystem6'
    // Start for Atomic SubSystem: '<S38>/Publish'
    // Start for MATLABSystem: '<S174>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_kr.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_kr.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_kr.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_kr.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_kr.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_6[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16122.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_kr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S174>/SinkBlock'
    // End of Start for SubSystem: '<S38>/Publish'

    // Start for Atomic SubSystem: '<S38>/Publish1'
    // Start for MATLABSystem: '<S175>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_gy.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_gy.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_gy.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_gy.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_gy.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_5[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16123.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_gy.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S175>/SinkBlock'
    // End of Start for SubSystem: '<S38>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem6'

    // Start for Enabled SubSystem: '<Root>/Subsystem7'
    // Start for Atomic SubSystem: '<S39>/Publish'
    // Start for MATLABSystem: '<S178>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_cg.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_cg.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_cg.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_cg.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_cg.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_4[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16135.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_cg.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S178>/SinkBlock'
    // End of Start for SubSystem: '<S39>/Publish'

    // Start for Atomic SubSystem: '<S39>/Publish1'
    // Start for MATLABSystem: '<S179>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_pd.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_pd.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_pd.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_pd.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_pd.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_3[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16136.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_pd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S179>/SinkBlock'
    // End of Start for SubSystem: '<S39>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem7'

    // Start for Enabled SubSystem: '<Root>/Subsystem8'
    // Start for Atomic SubSystem: '<S40>/Publish'
    // Start for MATLABSystem: '<S182>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_a0.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_a0.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_a0.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_a0.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_a0.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1p[i] = tmp_2[i];
    }

    tmp_1p[14] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16148.createPublisher(tmp_1p, 1);
    RflysimIndoorController_r201_DW.obj_a0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S182>/SinkBlock'
    // End of Start for SubSystem: '<S40>/Publish'

    // Start for Atomic SubSystem: '<S40>/Publish1'
    // Start for MATLABSystem: '<S183>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_oa.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_oa.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_oa.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_oa.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_oa.isInitialized = 1;
    for (i = 0; i < 41; i++) {
      RflysimIndoorController_r2018_B.cv3[i] = tmp_1[i];
    }

    RflysimIndoorController_r2018_B.cv3[41] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16149.createPublisher
      (RflysimIndoorController_r2018_B.cv3, 1);
    RflysimIndoorController_r201_DW.obj_oa.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S183>/SinkBlock'
    // End of Start for SubSystem: '<S40>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem8'

    // Start for Enabled SubSystem: '<Root>/Subsystem9'
    // Start for Atomic SubSystem: '<S41>/Publish'
    // Start for MATLABSystem: '<S186>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_by.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_by.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_by.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_by.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_by.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_1o[i] = tmp_0[i];
    }

    tmp_1o[15] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16161.createPublisher(tmp_1o, 1);
    RflysimIndoorController_r201_DW.obj_by.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S186>/SinkBlock'
    // End of Start for SubSystem: '<S41>/Publish'

    // Start for Atomic SubSystem: '<S41>/Publish1'
    // Start for MATLABSystem: '<S187>/SinkBlock'
    RflysimIndoorController_r201_DW.obj_du.matlabCodegenIsDeleted = true;
    RflysimIndoorController_r201_DW.obj_du.isInitialized = 0;
    RflysimIndoorController_r201_DW.obj_du.matlabCodegenIsDeleted = false;
    RflysimIndoorController_r201_DW.obj_du.isSetupComplete = false;
    RflysimIndoorController_r201_DW.obj_du.isInitialized = 1;
    for (i = 0; i < 42; i++) {
      RflysimIndoorController_r2018_B.cv2[i] = tmp[i];
    }

    RflysimIndoorController_r2018_B.cv2[42] = '\x00';
    Pub_RflysimIndoorController_r2018b_N12_V9_16162.createPublisher
      (RflysimIndoorController_r2018_B.cv2, 1);
    RflysimIndoorController_r201_DW.obj_du.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S187>/SinkBlock'
    // End of Start for SubSystem: '<S41>/Publish1'
    // End of Start for SubSystem: '<Root>/Subsystem9'

    // InitializeConditions for Memory: '<Root>/Memory4'
    RflysimIndoorController_r201_DW.Memory4_PreviousInput =
      RflysimIndoorController_r2018_P.Memory4_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory5'
    RflysimIndoorController_r201_DW.Memory5_PreviousInput =
      RflysimIndoorController_r2018_P.Memory5_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory6'
    RflysimIndoorController_r201_DW.Memory6_PreviousInput =
      RflysimIndoorController_r2018_P.Memory6_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory1'
    RflysimIndoorController_r201_DW.Memory1_PreviousInput =
      RflysimIndoorController_r2018_P.Memory1_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory2'
    RflysimIndoorController_r201_DW.Memory2_PreviousInput =
      RflysimIndoorController_r2018_P.Memory2_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory3'
    RflysimIndoorController_r201_DW.Memory3_PreviousInput =
      RflysimIndoorController_r2018_P.Memory3_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory10'
    RflysimIndoorController_r201_DW.Memory10_PreviousInput =
      RflysimIndoorController_r2018_P.Memory10_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory11'
    RflysimIndoorController_r201_DW.Memory11_PreviousInput =
      RflysimIndoorController_r2018_P.Memory11_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory12'
    RflysimIndoorController_r201_DW.Memory12_PreviousInput =
      RflysimIndoorController_r2018_P.Memory12_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory7'
    RflysimIndoorController_r201_DW.Memory7_PreviousInput =
      RflysimIndoorController_r2018_P.Memory7_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory8'
    RflysimIndoorController_r201_DW.Memory8_PreviousInput =
      RflysimIndoorController_r2018_P.Memory8_InitialCondition;

    // InitializeConditions for Memory: '<Root>/Memory9'
    RflysimIndoorController_r201_DW.Memory9_PreviousInput =
      RflysimIndoorController_r2018_P.Memory9_InitialCondition;

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S104>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S116>/Out1'
    RflysimIndoorController_r2018_B.In1_ks =
      RflysimIndoorController_r2018_P.Out1_Y0_c5;

    // End of SystemInitialize for SubSystem: '<S104>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe4'
    // SystemInitialize for Enabled SubSystem: '<S110>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S122>/Out1'
    RflysimIndoorController_r2018_B.In1_h3 =
      RflysimIndoorController_r2018_P.Out1_Y0_hx;

    // End of SystemInitialize for SubSystem: '<S110>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe4'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe8'
    // SystemInitialize for Enabled SubSystem: '<S114>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S126>/Out1'
    RflysimIndoorController_r2018_B.In1_lp =
      RflysimIndoorController_r2018_P.Out1_Y0_ac;

    // End of SystemInitialize for SubSystem: '<S114>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe8'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S105>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S117>/Out1'
    RflysimIndoorController_r2018_B.In1_n0 =
      RflysimIndoorController_r2018_P.Out1_Y0_bn;

    // End of SystemInitialize for SubSystem: '<S105>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S108>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S120>/Out1'
    RflysimIndoorController_r2018_B.In1_ox =
      RflysimIndoorController_r2018_P.Out1_Y0_kt;

    // End of SystemInitialize for SubSystem: '<S108>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe3'
    // SystemInitialize for Enabled SubSystem: '<S109>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S121>/Out1'
    RflysimIndoorController_r2018_B.In1_dj =
      RflysimIndoorController_r2018_P.Out1_Y0_cr;

    // End of SystemInitialize for SubSystem: '<S109>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe3'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe5'
    // SystemInitialize for Enabled SubSystem: '<S111>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S123>/Out1'
    RflysimIndoorController_r2018_B.In1_hr =
      RflysimIndoorController_r2018_P.Out1_Y0_os;

    // End of SystemInitialize for SubSystem: '<S111>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe5'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe10'
    // SystemInitialize for Enabled SubSystem: '<S106>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S118>/Out1'
    RflysimIndoorController_r2018_B.In1_fk =
      RflysimIndoorController_r2018_P.Out1_Y0_kn;

    // End of SystemInitialize for SubSystem: '<S106>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe10'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe11'
    // SystemInitialize for Enabled SubSystem: '<S107>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S119>/Out1'
    RflysimIndoorController_r2018_B.In1_er =
      RflysimIndoorController_r2018_P.Out1_Y0_ek;

    // End of SystemInitialize for SubSystem: '<S107>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe11'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe6'
    // SystemInitialize for Enabled SubSystem: '<S112>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S124>/Out1'
    RflysimIndoorController_r2018_B.In1_o =
      RflysimIndoorController_r2018_P.Out1_Y0_jz5;

    // End of SystemInitialize for SubSystem: '<S112>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe6'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe7'
    // SystemInitialize for Enabled SubSystem: '<S113>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S125>/Out1'
    RflysimIndoorController_r2018_B.In1_b1 =
      RflysimIndoorController_r2018_P.Out1_Y0_hm;

    // End of SystemInitialize for SubSystem: '<S113>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe7'

    // SystemInitialize for Atomic SubSystem: '<S17>/Subscribe9'
    // SystemInitialize for Enabled SubSystem: '<S115>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S127>/Out1'
    RflysimIndoorController_r2018_B.In1_a1 =
      RflysimIndoorController_r2018_P.Out1_Y0_lz;

    // End of SystemInitialize for SubSystem: '<S115>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S17>/Subscribe9'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe3'
    // SystemInitialize for Enabled SubSystem: '<S23>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S133>/Out1'
    RflysimIndoorController_r2018_B.In1_g =
      RflysimIndoorController_r2018_P.Out1_Y0_n;

    // End of SystemInitialize for SubSystem: '<S23>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe3'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe7'
    // SystemInitialize for Enabled SubSystem: '<S27>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S137>/Out1'
    RflysimIndoorController_r2018_B.In1_f =
      RflysimIndoorController_r2018_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S27>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe7'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe11'
    // SystemInitialize for Enabled SubSystem: '<S20>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S130>/Out1'
    RflysimIndoorController_r2018_B.In1_n =
      RflysimIndoorController_r2018_P.Out1_Y0_j;

    // End of SystemInitialize for SubSystem: '<S20>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe11'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S22>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S132>/Out1'
    RflysimIndoorController_r2018_B.In1_m =
      RflysimIndoorController_r2018_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S22>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe4'
    // SystemInitialize for Enabled SubSystem: '<S24>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S134>/Out1'
    RflysimIndoorController_r2018_B.In1_l =
      RflysimIndoorController_r2018_P.Out1_Y0_jt;

    // End of SystemInitialize for SubSystem: '<S24>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe4'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S18>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S128>/Out1'
    RflysimIndoorController_r2018_B.In1_d =
      RflysimIndoorController_r2018_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S18>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe9'
    // SystemInitialize for Enabled SubSystem: '<S29>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S139>/Out1'
    RflysimIndoorController_r2018_B.In1 =
      RflysimIndoorController_r2018_P.Out1_Y0_op;

    // End of SystemInitialize for SubSystem: '<S29>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe9'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe12'
    // SystemInitialize for Enabled SubSystem: '<S21>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S131>/Out1'
    RflysimIndoorController_r2018_B.In1_p =
      RflysimIndoorController_r2018_P.Out1_Y0_h;

    // End of SystemInitialize for SubSystem: '<S21>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe12'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe6'
    // SystemInitialize for Enabled SubSystem: '<S26>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S136>/Out1'
    RflysimIndoorController_r2018_B.In1_a4 =
      RflysimIndoorController_r2018_P.Out1_Y0_e;

    // End of SystemInitialize for SubSystem: '<S26>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe6'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe8'
    // SystemInitialize for Enabled SubSystem: '<S28>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S138>/Out1'
    RflysimIndoorController_r2018_B.In1_a =
      RflysimIndoorController_r2018_P.Out1_Y0_i;

    // End of SystemInitialize for SubSystem: '<S28>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe8'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe10'
    // SystemInitialize for Enabled SubSystem: '<S19>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S129>/Out1'
    RflysimIndoorController_r2018_B.In1_k =
      RflysimIndoorController_r2018_P.Out1_Y0_o;

    // End of SystemInitialize for SubSystem: '<S19>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe10'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe5'
    // SystemInitialize for Enabled SubSystem: '<S25>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S135>/Out1'
    RflysimIndoorController_r2018_B.In1_h =
      RflysimIndoorController_r2018_P.Out1_Y0_h3;

    // End of SystemInitialize for SubSystem: '<S25>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe5'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S56>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S80>/Out1'
    RflysimIndoorController_r2018_B.In1_a4x =
      RflysimIndoorController_r2018_P.Out1_Y0_eq;

    // End of SystemInitialize for SubSystem: '<S56>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe5'
    // SystemInitialize for Enabled SubSystem: '<S75>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S99>/Out1'
    RflysimIndoorController_r2018_B.In1_lm =
      RflysimIndoorController_r2018_P.Out1_Y0_ni;

    // End of SystemInitialize for SubSystem: '<S75>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe5'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe9'
    // SystemInitialize for Enabled SubSystem: '<S79>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S103>/Out1'
    RflysimIndoorController_r2018_B.In1_b =
      RflysimIndoorController_r2018_P.Out1_Y0_d;

    // End of SystemInitialize for SubSystem: '<S79>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe9'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe3'
    // SystemInitialize for Enabled SubSystem: '<S73>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S97>/Out1'
    RflysimIndoorController_r2018_B.In1_hw =
      RflysimIndoorController_r2018_P.Out1_Y0_ch;

    // End of SystemInitialize for SubSystem: '<S73>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe3'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe4'
    // SystemInitialize for Enabled SubSystem: '<S74>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S98>/Out1'
    RflysimIndoorController_r2018_B.In1_j =
      RflysimIndoorController_r2018_P.Out1_Y0_lc;

    // End of SystemInitialize for SubSystem: '<S74>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe4'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe7'
    // SystemInitialize for Enabled SubSystem: '<S77>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S101>/Out1'
    RflysimIndoorController_r2018_B.In1_fh =
      RflysimIndoorController_r2018_P.Out1_Y0_e2;

    // End of SystemInitialize for SubSystem: '<S77>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe7'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe13'
    // SystemInitialize for Enabled SubSystem: '<S60>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S84>/Out1'
    RflysimIndoorController_r2018_B.In1_fo =
      RflysimIndoorController_r2018_P.Out1_Y0_k;

    // End of SystemInitialize for SubSystem: '<S60>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe13'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe16'
    // SystemInitialize for Enabled SubSystem: '<S63>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S87>/Out1'
    RflysimIndoorController_r2018_B.In1_nu =
      RflysimIndoorController_r2018_P.Out1_Y0_l;

    // End of SystemInitialize for SubSystem: '<S63>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe16'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe18'
    // SystemInitialize for Enabled SubSystem: '<S65>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S89>/Out1'
    RflysimIndoorController_r2018_B.In1_bp =
      RflysimIndoorController_r2018_P.Out1_Y0_m1;

    // End of SystemInitialize for SubSystem: '<S65>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe18'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe14'
    // SystemInitialize for Enabled SubSystem: '<S61>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S85>/Out1'
    RflysimIndoorController_r2018_B.In1_ba =
      RflysimIndoorController_r2018_P.Out1_Y0_c;

    // End of SystemInitialize for SubSystem: '<S61>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe14'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe15'
    // SystemInitialize for Enabled SubSystem: '<S62>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S86>/Out1'
    RflysimIndoorController_r2018_B.In1_dh =
      RflysimIndoorController_r2018_P.Out1_Y0_an;

    // End of SystemInitialize for SubSystem: '<S62>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe15'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe17'
    // SystemInitialize for Enabled SubSystem: '<S64>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S88>/Out1'
    RflysimIndoorController_r2018_B.In1_mu =
      RflysimIndoorController_r2018_P.Out1_Y0_m;

    // End of SystemInitialize for SubSystem: '<S64>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe17'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S67>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S91>/Out1'
    RflysimIndoorController_r2018_B.In1_jl =
      RflysimIndoorController_r2018_P.Out1_Y0_f4;

    // End of SystemInitialize for SubSystem: '<S67>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe6'
    // SystemInitialize for Enabled SubSystem: '<S76>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S100>/Out1'
    RflysimIndoorController_r2018_B.In1_m1 =
      RflysimIndoorController_r2018_P.Out1_Y0_fb;

    // End of SystemInitialize for SubSystem: '<S76>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe6'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe10'
    // SystemInitialize for Enabled SubSystem: '<S57>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S81>/Out1'
    RflysimIndoorController_r2018_B.In1_jk =
      RflysimIndoorController_r2018_P.Out1_Y0_ng;

    // End of SystemInitialize for SubSystem: '<S57>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe10'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe11'
    // SystemInitialize for Enabled SubSystem: '<S58>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S82>/Out1'
    RflysimIndoorController_r2018_B.In1_mhe =
      RflysimIndoorController_r2018_P.Out1_Y0_kj;

    // End of SystemInitialize for SubSystem: '<S58>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe11'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe12'
    // SystemInitialize for Enabled SubSystem: '<S59>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S83>/Out1'
    RflysimIndoorController_r2018_B.In1_e =
      RflysimIndoorController_r2018_P.Out1_Y0_f;

    // End of SystemInitialize for SubSystem: '<S59>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe12'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe8'
    // SystemInitialize for Enabled SubSystem: '<S78>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S102>/Out1'
    RflysimIndoorController_r2018_B.In1_dh3 =
      RflysimIndoorController_r2018_P.Out1_Y0_fm;

    // End of SystemInitialize for SubSystem: '<S78>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe8'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe22'
    // SystemInitialize for Enabled SubSystem: '<S70>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S94>/Out1'
    RflysimIndoorController_r2018_B.In1_mh =
      RflysimIndoorController_r2018_P.Out1_Y0_k0;

    // End of SystemInitialize for SubSystem: '<S70>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe22'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe23'
    // SystemInitialize for Enabled SubSystem: '<S71>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S95>/Out1'
    RflysimIndoorController_r2018_B.In1_hk =
      RflysimIndoorController_r2018_P.Out1_Y0_mr;

    // End of SystemInitialize for SubSystem: '<S71>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe23'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe19'
    // SystemInitialize for Enabled SubSystem: '<S66>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S90>/Out1'
    RflysimIndoorController_r2018_B.In1_he =
      RflysimIndoorController_r2018_P.Out1_Y0_mb;

    // End of SystemInitialize for SubSystem: '<S66>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe19'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe20'
    // SystemInitialize for Enabled SubSystem: '<S68>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S92>/Out1'
    RflysimIndoorController_r2018_B.In1_ha =
      RflysimIndoorController_r2018_P.Out1_Y0_ap;

    // End of SystemInitialize for SubSystem: '<S68>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe20'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe21'
    // SystemInitialize for Enabled SubSystem: '<S69>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S93>/Out1'
    RflysimIndoorController_r2018_B.In1_fz =
      RflysimIndoorController_r2018_P.Out1_Y0_b;

    // End of SystemInitialize for SubSystem: '<S69>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe21'

    // SystemInitialize for Atomic SubSystem: '<S16>/Subscribe24'
    // SystemInitialize for Enabled SubSystem: '<S72>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S96>/Out1'
    RflysimIndoorController_r2018_B.In1_ai =
      RflysimIndoorController_r2018_P.Out1_Y0_jz;

    // End of SystemInitialize for SubSystem: '<S72>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S16>/Subscribe24'
  }
}

// Model terminate function
void RflysimIndoorController_r2018b_N12_V9_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S17>/Subscribe'
  // Terminate for MATLABSystem: '<S104>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_n0);

  // End of Terminate for SubSystem: '<S17>/Subscribe'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe4'
  // Terminate for MATLABSystem: '<S110>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_jh);

  // End of Terminate for SubSystem: '<S17>/Subscribe4'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe8'
  // Terminate for MATLABSystem: '<S114>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_n);

  // End of Terminate for SubSystem: '<S17>/Subscribe8'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe1'
  // Terminate for MATLABSystem: '<S105>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_lv);

  // End of Terminate for SubSystem: '<S17>/Subscribe1'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe2'
  // Terminate for MATLABSystem: '<S108>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_pf);

  // End of Terminate for SubSystem: '<S17>/Subscribe2'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe3'
  // Terminate for MATLABSystem: '<S109>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_mk);

  // End of Terminate for SubSystem: '<S17>/Subscribe3'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe5'
  // Terminate for MATLABSystem: '<S111>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_f);

  // End of Terminate for SubSystem: '<S17>/Subscribe5'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe10'
  // Terminate for MATLABSystem: '<S106>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_o);

  // End of Terminate for SubSystem: '<S17>/Subscribe10'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe11'
  // Terminate for MATLABSystem: '<S107>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_iw);

  // End of Terminate for SubSystem: '<S17>/Subscribe11'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe6'
  // Terminate for MATLABSystem: '<S112>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_k4n);

  // End of Terminate for SubSystem: '<S17>/Subscribe6'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe7'
  // Terminate for MATLABSystem: '<S113>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_k4);

  // End of Terminate for SubSystem: '<S17>/Subscribe7'

  // Terminate for Atomic SubSystem: '<S17>/Subscribe9'
  // Terminate for MATLABSystem: '<S115>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_m);

  // End of Terminate for SubSystem: '<S17>/Subscribe9'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe3'
  // Terminate for MATLABSystem: '<S23>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_b);

  // End of Terminate for SubSystem: '<Root>/Subscribe3'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe7'
  // Terminate for MATLABSystem: '<S27>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_k);

  // End of Terminate for SubSystem: '<Root>/Subscribe7'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe11'
  // Terminate for MATLABSystem: '<S20>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_j);

  // End of Terminate for SubSystem: '<Root>/Subscribe11'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe2'
  // Terminate for MATLABSystem: '<S22>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Subscribe2'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe4'
  // Terminate for MATLABSystem: '<S24>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_i);

  // End of Terminate for SubSystem: '<Root>/Subscribe4'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S18>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_aj);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe9'
  // Terminate for MATLABSystem: '<S29>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Subscribe9'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe12'
  // Terminate for MATLABSystem: '<S21>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_bb);

  // End of Terminate for SubSystem: '<Root>/Subscribe12'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe6'
  // Terminate for MATLABSystem: '<S26>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_ps);

  // End of Terminate for SubSystem: '<Root>/Subscribe6'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe8'
  // Terminate for MATLABSystem: '<S28>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_p);

  // End of Terminate for SubSystem: '<Root>/Subscribe8'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe10'
  // Terminate for MATLABSystem: '<S19>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_h);

  // End of Terminate for SubSystem: '<Root>/Subscribe10'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe5'
  // Terminate for MATLABSystem: '<S25>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_l);

  // End of Terminate for SubSystem: '<Root>/Subscribe5'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe1'
  // Terminate for MATLABSystem: '<S56>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_dy);

  // End of Terminate for SubSystem: '<S16>/Subscribe1'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe5'
  // Terminate for MATLABSystem: '<S75>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_g);

  // End of Terminate for SubSystem: '<S16>/Subscribe5'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe9'
  // Terminate for MATLABSystem: '<S79>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_hv);

  // End of Terminate for SubSystem: '<S16>/Subscribe9'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe3'
  // Terminate for MATLABSystem: '<S73>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_bs);

  // End of Terminate for SubSystem: '<S16>/Subscribe3'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe4'
  // Terminate for MATLABSystem: '<S74>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_ma);

  // End of Terminate for SubSystem: '<S16>/Subscribe4'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe7'
  // Terminate for MATLABSystem: '<S77>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_nq);

  // End of Terminate for SubSystem: '<S16>/Subscribe7'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe13'
  // Terminate for MATLABSystem: '<S60>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_eu);

  // End of Terminate for SubSystem: '<S16>/Subscribe13'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe16'
  // Terminate for MATLABSystem: '<S63>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_e);

  // End of Terminate for SubSystem: '<S16>/Subscribe16'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe18'
  // Terminate for MATLABSystem: '<S65>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_d);

  // End of Terminate for SubSystem: '<S16>/Subscribe18'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe14'
  // Terminate for MATLABSystem: '<S61>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_jr);

  // End of Terminate for SubSystem: '<S16>/Subscribe14'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe15'
  // Terminate for MATLABSystem: '<S62>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_fr);

  // End of Terminate for SubSystem: '<S16>/Subscribe15'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe17'
  // Terminate for MATLABSystem: '<S64>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_mh);

  // End of Terminate for SubSystem: '<S16>/Subscribe17'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe2'
  // Terminate for MATLABSystem: '<S67>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_pa);

  // End of Terminate for SubSystem: '<S16>/Subscribe2'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe6'
  // Terminate for MATLABSystem: '<S76>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_bbp);

  // End of Terminate for SubSystem: '<S16>/Subscribe6'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe10'
  // Terminate for MATLABSystem: '<S57>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_ot);

  // End of Terminate for SubSystem: '<S16>/Subscribe10'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe11'
  // Terminate for MATLABSystem: '<S58>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_c);

  // End of Terminate for SubSystem: '<S16>/Subscribe11'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe12'
  // Terminate for MATLABSystem: '<S59>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_fe);

  // End of Terminate for SubSystem: '<S16>/Subscribe12'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe8'
  // Terminate for MATLABSystem: '<S78>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_bm);

  // End of Terminate for SubSystem: '<S16>/Subscribe8'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe22'
  // Terminate for MATLABSystem: '<S70>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_al);

  // End of Terminate for SubSystem: '<S16>/Subscribe22'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe23'
  // Terminate for MATLABSystem: '<S71>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_bp);

  // End of Terminate for SubSystem: '<S16>/Subscribe23'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe19'
  // Terminate for MATLABSystem: '<S66>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_hm);

  // End of Terminate for SubSystem: '<S16>/Subscribe19'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe20'
  // Terminate for MATLABSystem: '<S68>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_av);

  // End of Terminate for SubSystem: '<S16>/Subscribe20'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe21'
  // Terminate for MATLABSystem: '<S69>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_hc);

  // End of Terminate for SubSystem: '<S16>/Subscribe21'

  // Terminate for Atomic SubSystem: '<S16>/Subscribe24'
  // Terminate for MATLABSystem: '<S72>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&RflysimIndoorController_r201_DW.obj_he);

  // End of Terminate for SubSystem: '<S16>/Subscribe24'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem'
  // Terminate for Atomic SubSystem: '<S30>/Publish'
  // Terminate for MATLABSystem: '<S142>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_o3);

  // End of Terminate for SubSystem: '<S30>/Publish'

  // Terminate for Atomic SubSystem: '<S30>/Publish1'
  // Terminate for MATLABSystem: '<S143>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_gn);

  // End of Terminate for SubSystem: '<S30>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem1'
  // Terminate for Atomic SubSystem: '<S31>/Publish'
  // Terminate for MATLABSystem: '<S146>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_hd);

  // End of Terminate for SubSystem: '<S31>/Publish'

  // Terminate for Atomic SubSystem: '<S31>/Publish1'
  // Terminate for MATLABSystem: '<S147>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_cp);

  // End of Terminate for SubSystem: '<S31>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem1'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem10'
  // Terminate for Atomic SubSystem: '<S32>/Publish'
  // Terminate for MATLABSystem: '<S150>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_j4t);

  // End of Terminate for SubSystem: '<S32>/Publish'

  // Terminate for Atomic SubSystem: '<S32>/Publish1'
  // Terminate for MATLABSystem: '<S151>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_hx);

  // End of Terminate for SubSystem: '<S32>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem10'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem11'
  // Terminate for Atomic SubSystem: '<S33>/Publish'
  // Terminate for MATLABSystem: '<S154>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_id);

  // End of Terminate for SubSystem: '<S33>/Publish'

  // Terminate for Atomic SubSystem: '<S33>/Publish1'
  // Terminate for MATLABSystem: '<S155>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_l0);

  // End of Terminate for SubSystem: '<S33>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem11'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem2'
  // Terminate for Atomic SubSystem: '<S34>/Publish'
  // Terminate for MATLABSystem: '<S158>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_cm);

  // End of Terminate for SubSystem: '<S34>/Publish'

  // Terminate for Atomic SubSystem: '<S34>/Publish1'
  // Terminate for MATLABSystem: '<S159>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_j4);

  // End of Terminate for SubSystem: '<S34>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem2'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem3'
  // Terminate for Atomic SubSystem: '<S35>/Publish'
  // Terminate for MATLABSystem: '<S162>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_bt);

  // End of Terminate for SubSystem: '<S35>/Publish'

  // Terminate for Atomic SubSystem: '<S35>/Publish1'
  // Terminate for MATLABSystem: '<S163>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_kb);

  // End of Terminate for SubSystem: '<S35>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem3'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem4'
  // Terminate for Atomic SubSystem: '<S36>/Publish'
  // Terminate for MATLABSystem: '<S166>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_fu);

  // End of Terminate for SubSystem: '<S36>/Publish'

  // Terminate for Atomic SubSystem: '<S36>/Publish1'
  // Terminate for MATLABSystem: '<S167>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_bc);

  // End of Terminate for SubSystem: '<S36>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem4'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem5'
  // Terminate for Atomic SubSystem: '<S37>/Publish'
  // Terminate for MATLABSystem: '<S170>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_k5);

  // End of Terminate for SubSystem: '<S37>/Publish'

  // Terminate for Atomic SubSystem: '<S37>/Publish1'
  // Terminate for MATLABSystem: '<S171>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_k0);

  // End of Terminate for SubSystem: '<S37>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem5'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem6'
  // Terminate for Atomic SubSystem: '<S38>/Publish'
  // Terminate for MATLABSystem: '<S174>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_kr);

  // End of Terminate for SubSystem: '<S38>/Publish'

  // Terminate for Atomic SubSystem: '<S38>/Publish1'
  // Terminate for MATLABSystem: '<S175>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_gy);

  // End of Terminate for SubSystem: '<S38>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem6'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem7'
  // Terminate for Atomic SubSystem: '<S39>/Publish'
  // Terminate for MATLABSystem: '<S178>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_cg);

  // End of Terminate for SubSystem: '<S39>/Publish'

  // Terminate for Atomic SubSystem: '<S39>/Publish1'
  // Terminate for MATLABSystem: '<S179>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_pd);

  // End of Terminate for SubSystem: '<S39>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem7'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem8'
  // Terminate for Atomic SubSystem: '<S40>/Publish'
  // Terminate for MATLABSystem: '<S182>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_a0);

  // End of Terminate for SubSystem: '<S40>/Publish'

  // Terminate for Atomic SubSystem: '<S40>/Publish1'
  // Terminate for MATLABSystem: '<S183>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_oa);

  // End of Terminate for SubSystem: '<S40>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem8'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem9'
  // Terminate for Atomic SubSystem: '<S41>/Publish'
  // Terminate for MATLABSystem: '<S186>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_by);

  // End of Terminate for SubSystem: '<S41>/Publish'

  // Terminate for Atomic SubSystem: '<S41>/Publish1'
  // Terminate for MATLABSystem: '<S187>/SinkBlock'
  RflysimIndoo_matlabCodegenHa_i1(&RflysimIndoorController_r201_DW.obj_du);

  // End of Terminate for SubSystem: '<S41>/Publish1'
  // End of Terminate for SubSystem: '<Root>/Subsystem9'
}

//
// File trailer for generated code.
//
// [EOF]
//
