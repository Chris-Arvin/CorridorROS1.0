#ifndef BEZIER_LOCAL_PLANNER_BEZIER_LOCAL_PLANNER_H
#define BEZIER_LOCAL_PLANNER_BEZIER_LOCAL_PLANNER_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include "bezier_local_planner/utils.h"

namespace bezier_local_planner
{
class BezierLocalPlanner
{
public:
  BezierLocalPlanner(base_local_planner::CostmapModel* costmap_model,
                     const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                     double circumscribed_radius, const BezierConfig& cfg, ros::NodeHandle nh);
  virtual ~BezierLocalPlanner();

  bool computeVelocityCommands(const Velocity& robot_vel, const Pose2D& robot_pose,
                               const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap, int size_x,
                               int size_y, double resolution, double origin_x, double origin_y, Velocity& cmd_vel);

private:
  /**
   * @brief Check whether the planned path is feasible or not.
   *
   * This method currently checks only that the path, or a part of the path is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @return \c true, if the robot footprint along the first part of the path intersects with
   *         any obstacle in the costmap, \c false otherwise.
   */
  bool isPathFeasible(const std::vector<Pose2D>& path);
  // publish multi bezier curves
  void publishBezierPaths(const std::vector<std::vector<Pose2D> >& bezier_paths, ros::Publisher publisher);
  // publish single bezier curve
  void publishBezierPath(const std::vector<Pose2D>& bezier_path, ros::Publisher publisher);
  // initialize the candidate targets in robot frame
  void initializeCandidateTargets(const double& forward_detect_latitude);
  // tranform the candidate targets from robot frame to map frame
  void transformCandidateTargets(const Pose2D& robot_pose, const std::vector<Pose2D>& global_plan);
  // go over all the targets to fit all paths
  void fitAllPaths(const Pose2D& robot_pose);
  // fit a path with bezier curve, constrained by the current pose p0 and target pose p3
  std::vector<Pose2D> fitPath(const Pose2D& p0, const Pose2D& p3);
  // transform a point from world resolution into map resolution
  void wolrd2map(const double& wx, const double& wy, int& mx, int& my, const double& resolution, const double& origin_x, const double& origin_y);
  // constrain a theta into [-pi, pi]
  double constrainTheta(double theta);
  // socre all the paths
  void scorePaths(const int& size_x, const int& size_y, const double& resolution, const double& origin_x, const double& origin_y, const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap);
  // score for a path
  double scorePath(const std::vector<Pose2D>& path, const int& size_x, const int& size_y, const double& resolution, const double& origin_x, const double& origin_y, const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap);
  // generate velocity with P controller
  Velocity generateVel(const Pose2D& robot_pose, const Velocity& robot_vel);

private:
  const BezierConfig* cfg_;  //!< Config class that stores and manages all related parameters

  base_local_planner::CostmapModel* costmap_model_;   //!< Pointer to the costmap model
  std::vector<geometry_msgs::Point> footprint_spec_;  //!< The specification of the footprint of the robot in world
                                                      //!< coordinates
  double inscribed_radius_;                           //!< The radius of the inscribed circle of the robot
  double circumscribed_radius_;                       //!< The radius of the circumscribed circle of the robot
  std::vector<Pose2D> abs_candidate_targets_;         //!< candidate targets in robot frame
  std::vector<Pose2D> candidate_targets_;             //!< candidate targets in map frame
  std::vector<std::vector<Pose2D>> feasible_candidate_paths_;   //!< feasible candidate paths withou collision
  std::vector<Pose2D> tracked_path_;                  //!< the path with lowest cost
  ros::Publisher all_candidate_paths_pub_, feasible_candidate_paths_pub_, tracked_path_pub_;
};

}  // namespace bezier_local_planner

#endif  // BEZIER_LOCAL_PLANNER_BEZIER_LOCAL_PLANNER_H