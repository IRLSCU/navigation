/*
 * latched_stop_rotate_controller.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#include <base_local_planner/latched_stop_rotate_controller.h>

#include <cmath>

#include <Eigen/Core>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_limits.h>

namespace base_local_planner {

LatchedStopRotateController::LatchedStopRotateController() {}

void LatchedStopRotateController::initialize(const std::string& name) {
    ros::NodeHandle private_nh("~/" + name);
    std::string param_name;
    private_nh.searchParam("latch_xy_goal_tolerance", param_name);
    private_nh.param(param_name, latch_xy_goal_tolerance_, false);
    rotating_to_goal_ = false;
  }

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true
 */
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util,
    tf::Stamped<tf::Pose> global_pose) {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance; //获取与目标点在xy平面内的可容忍误差

  //we assume the global goal is the last point in the global plan
  tf::Stamped<tf::Pose> goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.getOrigin().getX();
  double goal_y = goal_pose.getOrigin().getY();

  //check to see if we've reached the goal position
  //一般都为false，如果为true是什么意思呢？就是当进入xy_goal_tolerance范围内后会设置一个锁
  //此后即使在旋转调整yaw的过程中跳出xy_goal_tolerance，也不会进行xy上的调整。
  //设置为true时表示：如果到达容错距离内,机器人就会原地旋转；即使转动是会跑出容错距离外
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}


/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool LatchedStopRotateController::isGoalReached(LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper,
    tf::Stamped<tf::Pose> global_pose) {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;
  double rot_stopped_vel = planner_util->getCurrentLimits().rot_stopped_vel;
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;

  //copy over the odometry information
  nav_msgs::Odometry base_odom;
  odom_helper.getOdom(base_odom);

  //we assume the global goal is the last point in the global plan
  tf::Stamped<tf::Pose> goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.getOrigin().getX();
  double goal_y = goal_pose.getOrigin().getY();

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;
    }
    double goal_th = tf::getYaw(goal_pose.getRotation());
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //make sure that we're actually stopped before returning success
      if (base_local_planner::stopped(base_odom, rot_stopped_vel, trans_stopped_vel)) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief 
 * @param  global_pose      小车当前位资
 * @param  robot_vel        小车当前速度
 * @param  cmd_vel          待计算的小车速度控制指令
 * @param  acc_lim          小车加速度限制
 * @param  sim_period       前向模拟时间
 * @param  obstacle_check   回调函数
 * @return true 
 * @return false 
 */
bool LatchedStopRotateController::stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose,
    const tf::Stamped<tf::Pose>& robot_vel,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {

  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.

  //robot_vel.getOrigin().x() = 当前小车速度.linear.x?　
  //acc_lim[0]ｘ方向上的加速度限制、acc_lim[1]ｙ方向上的加速度限制
  //sign()来控制正向或者反向
  double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim[0] * sim_period));
  double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim[1] * sim_period));

  double vel_yaw = tf::getYaw(robot_vel.getRotation());　//获取旋转参数、acc_lim[2]旋转加速度先
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * sim_period));

  //we do want to check whether or not the command is valid
  double yaw = tf::getYaw(global_pose.getRotation());

  //调用DWAPlanner::checkTrajectory
  //Eigen::Vector3f(1,2,3)，列向量[1 2 3]
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
                                  Eigen::Vector3f(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));

  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd){
    ROS_DEBUG_NAMED("latched_stop_rotate", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }
  ROS_WARN("Stopping cmd in collision");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

/**
 * @brief 
 * @param  global_pose      小车当前位资
 * @param  robot_vel        小车当前速度
 * @param  goal_th          获得目标点位资的角度
 * @param  cmd_vel          待计算的小车速度控制指令
 * @param  acc_lim          小车加速度限制
 * @param  sim_period       前向模拟时间
 * @param  limits           参数配置的对象引用 base_local_planner::LocalPlannerLimits
 * @param  obstacle_check   调用DWAPlanner::checkTrajectory
 * @return true 
 * @return false 
 */
bool LatchedStopRotateController::rotateToGoal(
    const tf::Stamped<tf::Pose>& global_pose,
    const tf::Stamped<tf::Pose>& robot_vel,
    double goal_th,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    base_local_planner::LocalPlannerLimits& limits,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {
  double yaw = tf::getYaw(global_pose.getRotation());
  double vel_yaw = tf::getYaw(robot_vel.getRotation());
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  //获得最短的角度差异
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
  //获取能达到的最大旋转角速度
  double v_theta_samp = std::min(limits.max_rot_vel, std::max(limits.min_rot_vel, fabs(ang_diff)));

  //take the acceleration limits of the robot into account
  //考虑加速度的旋转角速度
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;
  //考虑加速度基础下，能达到的最大旋转角速度
  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

  v_theta_samp = std::min(limits.max_rot_vel, std::max(limits.min_rot_vel, v_theta_samp));

  if (ang_diff < 0) {
    v_theta_samp = - v_theta_samp;
  }

  //we still want to lay down the footprint of the robot and check if the action is legal
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
      Eigen::Vector3f(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw),
      Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

  if (valid_cmd) {
    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }
  ROS_WARN("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;

}

/**
 * @brief 
 * @param  cmd_vel          待输出的速度控制指令
 * @param  acc_lim          小车加速度限制
 * @param  sim_period       前向模拟时间
 * @param  planner_util     ase_local_planner::LocalPlannerUtil对象
 * @param  odom_helper_     base_local_planner::OdometryHelperRos对象，初始为"odom"
 * @param  global_pose      小车当前位资
 * @param  obstacle_check   回调函数，实则调用dp_->checkTrajectory(,,);
 * @return true 
 * @return false 
 */
bool LatchedStopRotateController::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper_,
    tf::Stamped<tf::Pose> global_pose,
    boost::function<bool (Eigen::Vector3f pos, // 对应_1
                          Eigen::Vector3f vel, // 对应_2
                          Eigen::Vector3f vel_samples /* 对应_3*/)> obstacle_check) {
  //we assume the global goal is the last point in the global plan
  tf::Stamped<tf::Pose> goal_pose;
  //获取局部目标点位姿
  if ( ! planner_util->getGoal(goal_pose)) {
    ROS_ERROR("Could not get goal pose");
    return false;
  }

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();　//获得参数配置信息

  //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
  //just rotate in place
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) {
    ROS_INFO("Goal position reached, stopping and turning in place");
    xy_tolerance_latch_ = true;
  }
  //check to see if the goal orientation has been reached
  double goal_th = tf::getYaw(goal_pose.getRotation());　//获得目标点位资的角度
  //获取小车当前位资与目标点位资的角度偏移
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);

  if (fabs(angle) <= limits.yaw_goal_tolerance) {　//角度差在可容忍范围内，则直接发布０速度指令
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    rotating_to_goal_ = false;
  } else {
    ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    tf::Stamped<tf::Pose> robot_vel;
    
    //基于里程计获取小车速度
    odom_helper_.getRobotVel(robot_vel);
    nav_msgs::Odometry base_odom;
    //获取里程计信息
    odom_helper_.getOdom(base_odom);

    //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, limits.rot_stopped_vel, limits.trans_stopped_vel)) {
      if ( ! stopWithAccLimits(
          global_pose,
          robot_vel,
          cmd_vel,
          acc_lim,
          sim_period,
          obstacle_check)) {
        ROS_INFO("Error when stopping.");
        return false;
      }
      ROS_DEBUG("Stopping...");
    }
    //if we're stopped... then we want to rotate to goal
    else {
      //set this so that we know its OK to be moving
      rotating_to_goal_ = true;
      if ( ! rotateToGoal(
          global_pose,
          robot_vel,
          goal_th,
          cmd_vel,
          acc_lim,
          sim_period,
          limits,
          obstacle_check)) {
        ROS_INFO("Error when rotating.");
        return false;
      }
      ROS_DEBUG("Rotating...");
    }
  }

  return true;

}


} /* namespace base_local_planner */
