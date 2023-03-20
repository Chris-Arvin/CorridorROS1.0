#include <chrono>
#include <pluginlib/class_list_macros.h>
#include "api2python/api2python_ros.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(api2python::API2PythonROS, nav_core::BaseLocalPlanner)

namespace api2python
{
API2PythonROS::API2PythonROS()
  : initialized_(false), size_x_(0), size_y_(0), charmap_(NULL),costmap_model_(NULL)
{
}

API2PythonROS::~API2PythonROS()
{
  if (costmap_model_)
    delete costmap_model_;
}

void API2PythonROS::freeMemory()
{
  if (charmap_)
  {
    for (int i = 0; i < size_x_; i++)
    {
      delete[] charmap_[i];
      charmap_[i] = NULL;
    }

    delete[] charmap_;
    charmap_ = NULL;
  }
}

void API2PythonROS::allocateMemory()
{
  assert(charmap_ == NULL);

  charmap_ = new unsigned char*[size_x_];
  for (int i = 0; i < size_x_; i++)
    charmap_[i] = new unsigned char[size_y_];
}

void API2PythonROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if (!initialized_)
  {
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle private_nh("~/" + name);

    // get parameters of APIConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(private_nh);

    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();  // locking should be done in MoveBase.

    costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_;
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint
    // vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("api_global_plan", 1);
    python_client= private_nh.serviceClient<api2python::api_info>("/local_plan");  


    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("api_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("api_local_planner has already been initialized, doing nothing.");
  }
}

bool API2PythonROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if (!initialized_)
  {
    ROS_ERROR("api_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  global_goal_ = global_plan_.back();

  return true;
}

bool API2PythonROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // check if plugin is initialized
  if (!initialized_)
  {
    ROS_ERROR("api_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // Get current robot pose
  geometry_msgs::PoseStamped robot_pose_stamped;
  costmap_ros_->getRobotPose(robot_pose_stamped);

  // Get current robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  geometry_msgs::Twist robot_vel_stamped;
  robot_vel_stamped.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_stamped.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

  // Set map
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = global_frame_;
  map.header.stamp = ros::Time::now();
  map.info.width = costmap_->getSizeInCellsX();
  map.info.height = costmap_->getSizeInCellsY();
  map.info.origin.position.x = costmap_->getOriginX();
  map.info.origin.position.y = costmap_->getOriginY();
  map.info.resolution = costmap_->getResolution();

  char* cost_translation_table_ = new char[256];
  // special values:
  cost_translation_table_[0] = 0;  // NO obstacle
  cost_translation_table_[253] = 99;  // INSCRIBED obstacle 内切障碍物
  cost_translation_table_[254] = 100;  // LETHAL obstacle 致命的障碍物
  cost_translation_table_[255] = -1;  // UNKNOWN

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++)
  {
    cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
  }


  map.data.resize(map.info.width * map.info.height);
  unsigned char* data = costmap_->getCharMap();
  for (unsigned int i = 0; i < map.data.size(); i++)
  {
    map.data[i] = cost_translation_table_[ data[ i ]];
  }

  // Set reference path
  nav_msgs::Path reference_path;
  reference_path.header.frame_id = global_frame_;
  reference_path.header.stamp = ros::Time::now();
  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose_stamped, global_plan_);
  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  double max_global_plan_lookahead_dist = 3.0;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose_stamped, *costmap_, global_frame_,
                           max_global_plan_lookahead_dist, transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }
  for (int i = 0; i < (int)transformed_plan.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = transformed_plan[i].pose.position.x;
    pose.pose.position.y = transformed_plan[i].pose.position.y;
    reference_path.poses.push_back(pose);
  }
  publishGlobalPlan(transformed_plan);
  
  // robot_pose_stamped
  srv.request.robot_pose = robot_pose_stamped;
  srv.request.robot_vel= robot_vel_stamped; 
  srv.request.reference_path = reference_path;
  srv.request.map = map;
  if (python_client.call(srv)){
    cmd_vel = srv.response.cmd;
    return true;
  }
  else{
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }
}



bool API2PythonROS::isGoalReached()
{
  // check if plugin is initialized
  if (!initialized_)
  {
    ROS_ERROR("api2python_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // Get current robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  double dx = robot_pose.pose.position.x - global_goal_.pose.position.x;
  double dy = robot_pose.pose.position.y - global_goal_.pose.position.y;
  if (hypot(dx, dy) < cfg_.xy_goal_tolerance)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  else
    return false;
}

bool API2PythonROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;

  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform =
        tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;

    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}

bool API2PythonROS::transformGlobalPlan(const tf2_ros::Buffer& tf,
                                        const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                        const geometry_msgs::PoseStamped& global_pose,
                                        const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                                        double max_plan_length,
                                        std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                                        int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform =
        tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                           plan_pose.header.frame_id, ros::Duration(cfg_.transform_tolerance));

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    // we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                             // located on the border of the local costmap

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    // we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for (int j = 0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist)  // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }

    geometry_msgs::PoseStamped newer_pose;

    double plan_length = 0;  // check cumulative Euclidean distance along the plan

    // now we'll transform until points are outside of our distance threshold
    while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold &&
           (max_plan_length <= 0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // caclulate distance to previous pose
      if (i > 0 && max_plan_length > 0)
      {
        double dx = global_plan[i].pose.position.x - global_plan[i - 1].pose.position.x;
        double dy = global_plan[i].pose.position.y - global_plan[i - 1].pose.position.y;
        plan_length += hypot(dx, dy);
      }

      ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error
    // >>0) the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx)
        *current_goal_idx = int(global_plan.size()) - 1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx)
        *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
    }

    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global)
      *tf_plan_to_global = plan_to_global_transform;
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                global_plan[0].header.frame_id.c_str());

    return false;
  }
  return true;
}

void API2PythonROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_.map_frame;
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = global_plan;
  global_plan_pub_.publish(gui_path);
}

}  // namespace api_planner