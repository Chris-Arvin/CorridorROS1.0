#ifndef _API2PYTHON_UTILS_H_
#define _API2PYTHON_UTILS_H_

#include <cmath>
#include <ros/ros.h>

namespace api2python
{
/**
 * Normalizes angle to be in the range of [-pi, pi]
 * @param angle to be normalized
 * @return normalized angle
 */
inline double NormalizeAngle(double angle)
{
  while (angle < -M_PI)
  {
    if (angle < -(2.0 * M_PI))
    {
      angle += (int)(angle / -(2.0 * M_PI)) * (2.0 * M_PI);
    }
    else
    {
      angle += (2.0 * M_PI);
    }
  }

  while (angle > M_PI)
  {
    if (angle > (2.0 * M_PI))
    {
      angle -= (int)(angle / (2.0 * M_PI)) * (2.0 * M_PI);
    }
    else
    {
      angle -= (2.0 * M_PI);
    }
  }

  assert(angle >= -M_PI && angle <= M_PI);

  return angle;
}


class Pose2D
{
public:
  Pose2D()
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
  }

  Pose2D(double _x, double _y, double _theta)
  {
    x = _x;
    y = _y;
    theta = _theta;
  }

  double x;
  double y;
  double theta;
};

class Velocity
{
public:
  Velocity()
  {
    v = 0.0;
    omega = 0.0;
  }

  Velocity(double _v, double _omega)
  {
    v = _v;
    omega = _omega;
  }

  double v;
  double omega;
};

class Trajectory
{
public:
  Trajectory(){
  }
  void addPose(Pose2D pos){
    traj.push_back(pos);
  }
  void setVelocity(Velocity vel){
    cmd_vel = vel;
  }
  std::vector<Pose2D> getPoses(){
    return traj;
  }
  Velocity getVelocity(){
    return cmd_vel;
  }
  Pose2D getEndpose(){
    return traj.back();
  }
private:
  std::vector<Pose2D> traj;
  Velocity cmd_vel;  
};

class APIConfig
{
public:
  std::string odom_topic;      //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame
  double xy_goal_tolerance;    //!< The tolerance in meters for the controller in the x & y distance when achieving a
                               //!< goal
  double transform_tolerance;  //<! Tolerance when querying the TF Tree for a transformation (seconds)

  // Robot related parameters
  double max_vel_x;      //!< Maximum translational velocity of the robot
  double min_vel_x;      //!< Minimum linear velocity of the robot
  double max_vel_theta;  //!< Maximum angular velocity of the robot
  double min_vel_theta;  //!< Minimum angular velocity of the robot
  double acc_lim_x;      //!< Maximum translational acceleration of the robot
  double acc_lim_theta;  //!< Maximum angular acceleration of the robot
  double control_period;

  // DWA
  int sim_time_samples;
  int vx_samples;   //!< The number of samples to use when exploring the x velocity space
  int vth_samples;  //!< The number of samples to use when exploring the theta velocity space

  double path_distance_bias;  //!< The weighting for how much the controller should stay close to the path it was
                              //!< given
  double goal_distance_bias;  //!< The weighting for how much the controller should attempt to reach its local goal,
                              //!< also controls speed
  double occdist_scale;       //!< The weighting for how much the controller should attempt to avoid obstacles

  APIConfig()
  {
    odom_topic = "odom";
    map_frame = "map";
    xy_goal_tolerance = 0.2;
    transform_tolerance = 0.5;
    
    // Robot
    max_vel_x = 1;
    min_vel_x = -1;
    max_vel_theta = 1.2;
    min_vel_theta = -1.2;
    acc_lim_x = 0.5;
    acc_lim_theta = 0.5;
    control_period = 0.2;

    // DWA
    sim_time_samples = 10;
    vx_samples = 10;
    vth_samples = 20;
    path_distance_bias = 32.0;
    goal_distance_bias = 24.0;
    occdist_scale = 0.01;
  }

  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
  {
    nh.param("odom_topic", odom_topic, odom_topic);
    nh.param("xy_goal_tolerance", xy_goal_tolerance, xy_goal_tolerance);
    nh.param("transform_tolerance", transform_tolerance, transform_tolerance);

    // Robot
    nh.param("max_vel_x", max_vel_x, max_vel_x);
    nh.param("min_vel_x", min_vel_x, min_vel_x);
    nh.param("max_vel_theta", max_vel_theta, max_vel_theta);
    nh.param("min_vel_theta", min_vel_theta, min_vel_theta);
    nh.param("acc_lim_x", acc_lim_x, acc_lim_x);
    nh.param("acc_lim_theta", acc_lim_theta, acc_lim_theta);
    nh.param("control_period", control_period, control_period);

    // DWA
    nh.param("sim_time_samples", sim_time_samples, sim_time_samples);
    nh.param("vx_samples", vx_samples, vx_samples);
    nh.param("vth_samples", vth_samples, vth_samples);
    nh.param("path_distance_bias", path_distance_bias, path_distance_bias);
    nh.param("goal_distance_bias", goal_distance_bias, goal_distance_bias);
    nh.param("occdist_scale", occdist_scale, occdist_scale);
  }
};

}  // namespace dwa_planner

#endif  // _DWA_PLANNER_UTILS_H_