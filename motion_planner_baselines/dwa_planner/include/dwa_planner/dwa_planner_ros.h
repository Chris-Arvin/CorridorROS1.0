#ifndef _DWA_PLANNER_ROS_H_
#define _DWA_PLANNER_ROS_H_

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include "dwa_planner/utils.h"
#include "dwa_planner/dwa_planner.h"

namespace dwa_planner
{
class DWAPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief Default constructor of the dwa plugin
   */
  DWAPlannerROS();

  /**
   * @brief  Destructor of the plugin
   */
  ~DWAPlannerROS();

  /**
   * @brief Initializes the dwa plugin
   * @param name The name of the instance
   * @param tf Pointer to a tf buffer
   * @param costmap_ros Cost map representing occupied and free space
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the dwa local planner is following
   * @param orig_global_plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the
   * base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Check if the goal pose has been achieved
   *
   * The actual check is performed in computeVelocityCommands().
   * Only the status flag is checked here.
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

private:
  void freeMemory();
  void allocateMemory();
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

  /**
   * @brief Prune global plan such that already passed poses are cut off
   *
   * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
   * If no valid transformation can be found, the method returns \c false.
   * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
   * If no pose within the specified treshold \c dist_behind_robot can be found,
   * nothing will be pruned and the method returns \c false.
   * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will
   * be pruned.
   * @param tf A reference to a tf buffer
   * @param global_pose The global pose of the robot
   * @param[in,out] global_plan The plan to be transformed
   * @param dist_behind_robot Distance behind the robot that should be kept [meters]
   * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found
   * inside the threshold
   */
  bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                       std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot = 1);

  /**
   * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
   *
   * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
   * such that the index of the current goal pose is returned as well as
   * the transformation between the global plan and the planning frame.
   * @param tf A reference to a tf buffer
   * @param global_plan The plan to be transformed
   * @param global_pose The global pose of the robot
   * @param costmap A reference to the costmap being used so the window size for transforming can be computed
   * @param global_frame The frame to transform the plan to
   * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0:
   * disabled; the length is also bounded by the local costmap size!]
   * @param[out] transformed_plan Populated with the transformed plan
   * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
   * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
   * @return \c true if the global plan is transformed, \c false otherwise
   */
  bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame, double max_plan_length,
                           std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx = NULL,
                           geometry_msgs::TransformStamped* tf_plan_to_global = NULL) const;

private:
  // external objects (store weak pointers)
  costmap_2d::Costmap2DROS* costmap_ros_;  //!< Pointer to the costmap ros wrapper, received from the navigation stack
  costmap_2d::Costmap2D* costmap_;         //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  tf2_ros::Buffer* tf_;                    //!< pointer to tf buffer

  std::vector<geometry_msgs::PoseStamped> global_plan_;  //!< Store the current global plan

  base_local_planner::OdometryHelperRos odom_helper_;  //!< Provides an interface to receive the current velocity from
                                                       //!< the robot

  geometry_msgs::PoseStamped global_goal_;
  ros::Publisher global_plan_pub_;
  bool initialized_;

  std::vector<geometry_msgs::Point> footprint_spec_;  //!< Store the footprint of the robot
  base_local_planner::CostmapModel* costmap_model_;
  double robot_inscribed_radius_;     //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius;  //!< The radius of the circumscribed circle of the robot

  std::string global_frame_;      //!< The frame in which the controller will run
  std::string robot_base_frame_;  //!< Used as the base frame id of the robot
  std::string name_;              //!< For use with the ros nodehandle
  std::string odom_topic_;

  unsigned char** charmap_;
  int size_x_, size_y_;
  DWAPlanner* planner_;
  DWAConfig cfg_;
};

}  // namespace dwa_planner

#endif  // _DWA_PLANNER_ROS_H_