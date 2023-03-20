#ifndef TIME_BOUNDED_LATTICE_PLANNER_UTILS_H
#define TIME_BOUNDED_LATTICE_PLANNER_UTILS_H

#include <ros/ros.h>

namespace time_bounded_lattice
{
inline double CubicBezierPoint(double P0, double P1, double P2, double P3, double t)
{
  double value = pow(1 - t, 3) * P0 + 3 * pow(1 - t, 2) * t * P1 + 3 * (1 - t) * pow(t, 2) * P2 + pow(t, 3) * P3;
  return value;
}

inline double CubicBezierFirstOrder(double P0, double P1, double P2, double P3, double t)
{
  double value = 3 * pow(1 - t, 2) * (P1 - P0) + 6 * (1 - t) * t * (P2 - P1) + 3 * pow(t, 2) * (P3 - P2);
  return value;
}


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

/**
 * @brief Return the average angle of an arbitrary number of given angles [rad]
 * @param angles vector containing all angles
 * @return average / mean angle, that is normalized to [-pi, pi]
 */
inline double AverageAngles(const std::vector<double>& angles)
{
  double x = 0, y = 0;
  for (std::vector<double>::const_iterator it = angles.begin(); it != angles.end(); ++it)
  {
    x += cos(*it);
    y += sin(*it);
  }
  if (x == 0 && y == 0)
    return 0;
  else
    return std::atan2(y, x);
}

class Point2D
{
public:
  Point2D()
  {
    x = 0.0;
    y = 0.0;
  }

  Point2D(double _x, double _y)
  {
    x = _x;
    y = _y;
  }

  double x;
  double y;
};

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


class BezierConfig
{
public:
  std::string odom_topic;      //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame;       //!< Global planning frame
  double xy_goal_tolerance;    //!< The tolerance in meters for the controller in the x & y distance when achieving a
                               //!< goal
  double transform_tolerance;  //<! Tolerance when querying the TF Tree for a transformation (seconds)
  double max_global_plan_lookahead_dist;

  // Robot related parameters
  double max_vel_x;      //!< Maximum translational velocity of the robot
  double min_vel_x;      //!< Minimum linear velocity of the robot
  double max_vel_theta;  //!< Maximum angular velocity of the robot
  double min_vel_theta;  //!< Minimum angular velocity of the robot
  double acc_lim_x;      //!< Maximum translational acceleration of the robot
  double acc_lim_theta;  //!< Maximum angular acceleration of the robot
  double control_period;

  // Weights
  double path_distance_bias;  //!< The weighting for how much the controller should stay close to the path it was
                              //!< given
  double goal_distance_bias;  //!< The weighting for how much the controller should attempt to reach its local goal,
                              //!< also controls speed
  double occdist_scale;       //!< The weighting for how much the controller should attempt to avoid obstacles

  BezierConfig()
  {
    odom_topic = "odom";
    map_frame = "map";
    xy_goal_tolerance = 0.2;
    transform_tolerance = 0.5;
    max_global_plan_lookahead_dist = 3.0;

    // Robot
    max_vel_x = 1;
    min_vel_x = -1;
    max_vel_theta = 1.2;
    min_vel_theta = -1.2;
    acc_lim_x = 0.5;
    acc_lim_theta = 0.5;
    control_period = 0.2;

    // Weights
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
    nh.param("max_global_plan_lookahead_dist", max_global_plan_lookahead_dist, max_global_plan_lookahead_dist);

    // Robot
    nh.param("max_vel_x", max_vel_x, max_vel_x);
    nh.param("min_vel_x", min_vel_x, min_vel_x);
    nh.param("max_vel_theta", max_vel_theta, max_vel_theta);
    nh.param("min_vel_theta", min_vel_theta, min_vel_theta);
    nh.param("acc_lim_x", acc_lim_x, acc_lim_x);
    nh.param("acc_lim_theta", acc_lim_theta, acc_lim_theta);
    nh.param("control_period", control_period, control_period);

    // Weights
    nh.param("path_distance_bias", path_distance_bias, path_distance_bias);
    nh.param("goal_distance_bias", goal_distance_bias, goal_distance_bias);
    nh.param("occdist_scale", occdist_scale, occdist_scale);
  }
};

}  // namespace time_bounded_lattice

#endif  // TIME_BOUNDED_LATTICE_PLANNER_UTILS_H