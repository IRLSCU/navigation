/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/


#ifndef __base_local_planner__LOCALPLANNERLIMITS_H__
#define __base_local_planner__LOCALPLANNERLIMITS_H__

#include <Eigen/Core>

namespace base_local_planner
{
class LocalPlannerLimits
{
public:

  //DWA采用的是绝对值
  double max_trans_vel;　//器人最大平移速度的绝对值，单位为 m/s
  double min_trans_vel; //机器人最小平移速度的绝对值，单位为 m/s

  double max_vel_x; //x方向最大线速度绝对值，单位:米/秒
  double min_vel_x; //x方向最小线速度绝对值，单位:米/秒。如果为负值表示可以后退

  double max_vel_y; //y方向最大线速度绝对值，单位:米/秒
  double min_vel_y; //y方向最小线速度绝对值，单位:米/秒

  double max_rot_vel; //机器人的最大旋转角速度的绝对值，单位为 rad/s 
  double min_rot_vel; //机器人的最小旋转角速度的绝对值，单位为 rad/s

  double acc_lim_x; //机器人在x方向的加速度限制，单位为 meters/sec^2
  double acc_lim_y;　//机器人在y方向的加速度限制，单位为 meters/sec^2
  double acc_lim_theta; //机器人的旋转加速度限制，单位为 rad/sec^2
  double acc_limit_trans;
//  double jerk_lim_trans;
//  double jerk_lim_rot;
  bool   prune_plan;
  double xy_goal_tolerance; //到到目标点时，控制器在x和y方向上的容差（tolerence）（米）。即：到达目标点时,在xy平面内与目标点的距离误差
  double yaw_goal_tolerance; //到达目标点时，控制器在偏航/旋转时的弧度容差(tolerance)。即：到达目标点时偏行角允许的误差，单位弧度

  double trans_stopped_vel; //机器人被认属于“停止”状态时的平移速度。如果机器人的速度低于该值，则认为机器人已停止。单位为 m/s
  double rot_stopped_vel; //机器人被认属于“停止”状态时的旋转速度。单位为 rad/s

  bool   restore_defaults;

  LocalPlannerLimits() {}

  LocalPlannerLimits(
      double nmax_trans_vel,
      double nmin_trans_vel,
      double nmax_vel_x,
      double nmin_vel_x,
      double nmax_vel_y,
      double nmin_vel_y,
      double nmax_rot_vel,
      double nmin_rot_vel,
      double nacc_lim_x,
      double nacc_lim_y,
      double nacc_lim_theta,
      double nacc_limit_trans,
      double nxy_goal_tolerance,
      double nyaw_goal_tolerance,
//      double njerk_lim_trans = -1,
//      double njerk_lim_rot = -1,
      bool   nprune_plan = true,
      double ntrans_stopped_vel = 0.1,
      double nrot_stopped_vel = 0.1):
        max_trans_vel(nmax_trans_vel),
        min_trans_vel(nmin_trans_vel),
        max_vel_x(nmax_vel_x),
        min_vel_x(nmin_vel_x),
        max_vel_y(nmax_vel_y),
        min_vel_y(nmin_vel_y),
        max_rot_vel(nmax_rot_vel),
        min_rot_vel(nmin_rot_vel),
        acc_lim_x(nacc_lim_x),
        acc_lim_y(nacc_lim_y),
        acc_lim_theta(nacc_lim_theta),
        acc_limit_trans(nacc_limit_trans),
//        jerk_lim_trans(njerk_lim_trans),
//        jerk_lim_rot(njerk_lim_rot),
        prune_plan(nprune_plan),
        xy_goal_tolerance(nxy_goal_tolerance),
        yaw_goal_tolerance(nyaw_goal_tolerance),
        trans_stopped_vel(ntrans_stopped_vel),
        rot_stopped_vel(nrot_stopped_vel) {}

  ~LocalPlannerLimits() {}

  /**
   * @brief  Get the acceleration limits of the robot
   * @return  The acceleration limits of the robot
   */
  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

};

}
#endif // __LOCALPLANNERLIMITS_H__
