#include "dwa_planner/dwa_planner.h"
#include <Eigen/Core>

namespace dwa_planner
{

DWAPlanner::DWAPlanner(base_local_planner::CostmapModel* costmap_model,
                       const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                       double circumscribed_radius, const DWAConfig& cfg, ros::NodeHandle nh)
  : costmap_model_(costmap_model)
  , footprint_spec_(footprint_spec)
  , inscribed_radius_(inscribed_radius)
  , circumscribed_radius_(circumscribed_radius)
  , cfg_(&cfg)
{
  candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("dwa_candidate_paths", 1);
  following_traj_pub_ = nh.advertise<nav_msgs::Path>("dwa_following_traj", 1);

}

DWAPlanner::~DWAPlanner()
{
}

bool DWAPlanner::computeVelocityCommands(const Velocity& robot_vel, const Pose2D& robot_pose,
                                         const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap,
                                         int size_x, int size_y, double resolution, double origin_x, double origin_y,
                                         Velocity& cmd_vel)
{
  // sample velocities

  std::vector<Velocity> sample_vels;
  if (!samplePotentialVels(robot_vel,sample_vels)){
    return false;
  }
  std::vector<Velocity>::iterator it = sample_vels.begin();
  std::vector<std::vector<Pose2D>> path_all;  // record potential path
  Trajectory traj_followed;
  double min_score=1e10;
  while (true){
    // stop condition
    if (it==sample_vels.end()){
      publishCandidatePaths(path_all);
      publishFinalTraj(traj_followed);
      return true;
    }
    Trajectory Traj = generateTrajectory(robot_vel, robot_pose, *it);
    if (!isPathFeasible(Traj.getPoses())){
      it++;
      continue;
    }
    double score = scoreTrajectory(Traj, size_x, size_y, resolution, origin_x, origin_y, global_plan, costmap);
    if (score<min_score){
      min_score=score;
      cmd_vel = Traj.getVelocity();
      traj_followed = Traj;
    }
    path_all.push_back(Traj.getPoses());
    it++;
  }
  return false;
}

double DWAPlanner::scoreTrajectory(Trajectory& Traj, const int& size_x, const int& size_y, const double& resolution, const double& origin_x, const double& origin_y, const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap){
  // cost1: occupy
  const std::vector<Pose2D>& path = Traj.getPoses();
  int mx,my;
  int occupy = 0;
  for(std::vector<Pose2D>::const_iterator it=path.begin(); it!=path.end(); it++){
    wolrd2map(it->x, it->y, mx, my, resolution, origin_x, origin_y);
    occupy = std::max(costmap[mx][my]-0,occupy);
  }
  // cost2: distance to local target pose
  Pose2D endpose = Traj.getEndpose();
  Pose2D local_endpose = global_plan.back();
  double dis2end = sqrt(pow(endpose.x-local_endpose.x,2)+pow(endpose.y-local_endpose.y,2));
  // cost3: distance to reference path
  double dis2path = 1e10;
  for(std::vector<Pose2D>::const_iterator it=global_plan.begin(); it!=global_plan.end(); it++){
    dis2path = std::min(std::sqrt(std::pow(endpose.x-it->x,2)+std::pow(endpose.y-it->y,2)),dis2path);
  }
  return cfg_->occdist_scale*occupy + cfg_->goal_distance_bias*dis2end + cfg_->path_distance_bias*dis2path;
}

void DWAPlanner::wolrd2map(const double& wx, const double& wy, int& mx, int& my, const double& resolution, const double& origin_x, const double& origin_y){
  mx = int((wx-origin_x)/resolution);
  my = int((wy-origin_y)/resolution);
}

Trajectory DWAPlanner::generateTrajectory(const Velocity& robot_vel, const Pose2D& robot_pose, const Velocity& sample_vel){
  Trajectory Traj;
  Pose2D pose = robot_pose;
  Velocity vel = robot_vel;
  // simulate for sim_time_samples times
  for(int i=0; i<cfg_->sim_time_samples; i++){
    vel = computeNewVelocities(sample_vel, vel);
    pose = computeNewPose(pose,vel);
    if (i==0){
      Traj.setVelocity(vel);
    }
    Traj.addPose(pose);
  }
  return Traj;
}

Pose2D DWAPlanner::computeNewPose(const Pose2D &pos, const Velocity &vel){
  Pose2D new_pos;
  new_pos.x = pos.x + vel.v*cos(pos.theta)*cfg_->control_period;
  new_pos.y = pos.y + vel.v*sin(pos.theta)*cfg_->control_period;
  new_pos.theta = pos.theta +  vel.omega*cfg_->control_period;
  return new_pos;
}

Velocity DWAPlanner::computeNewVelocities(const Velocity &sample_target_vel, const Velocity &vel){
  Velocity new_vel;
  // v
  if (sample_target_vel.v<vel.omega){
    new_vel.v = std::max(sample_target_vel.v, vel.v-cfg_->acc_lim_x*cfg_->control_period);
  }
  else{
    new_vel.v = std::min(sample_target_vel.v, vel.v+cfg_->acc_lim_x*cfg_->control_period);
  }
  // w
  if (sample_target_vel.omega<vel.omega){
    new_vel.omega = std::max(sample_target_vel.omega, vel.omega-cfg_->acc_lim_theta*cfg_->control_period);
  }
  else{
    new_vel.omega = std::min(sample_target_vel.omega, vel.omega+cfg_->acc_lim_theta*cfg_->control_period);
  }
  return new_vel;
}

bool DWAPlanner::samplePotentialVels(const Velocity& robot_vel, std::vector<Velocity> & sample_vels)
{
  // double min_vel_x = std::max(cfg_->min_vel_x, robot_vel.v-cfg_->acc_lim_x*cfg_->control_period*cfg_->sim_time_samples);
  // double max_vel_x = std::min(cfg_->max_vel_x, robot_vel.v+cfg_->acc_lim_x*cfg_->control_period*cfg_->sim_time_samples);
  // double min_vel_theta = std::max(cfg_->min_vel_theta, robot_vel.omega-cfg_->acc_lim_theta*cfg_->control_period*cfg_->sim_time_samples);
  // double max_vel_theta = std::min(cfg_->max_vel_theta, robot_vel.omega+cfg_->acc_lim_theta*cfg_->control_period*cfg_->sim_time_samples);
  double min_vel_x = std::max(cfg_->min_vel_x, robot_vel.v-cfg_->acc_lim_x*cfg_->control_period);
  double max_vel_x = std::min(cfg_->max_vel_x, robot_vel.v+cfg_->acc_lim_x*cfg_->control_period);
  double min_vel_theta = std::max(cfg_->min_vel_theta, robot_vel.omega-cfg_->acc_lim_theta*cfg_->control_period);
  double max_vel_theta = std::min(cfg_->max_vel_theta, robot_vel.omega+cfg_->acc_lim_theta*cfg_->control_period);
  for (double v=min_vel_x; v<=max_vel_x; v+=(max_vel_x-min_vel_x)/cfg_->vx_samples){
    for (double w=min_vel_theta; w<=max_vel_theta; w+=(max_vel_theta-min_vel_theta)/cfg_->vth_samples){
      sample_vels.push_back(Velocity(v,w));
    }
  }
  if (sample_vels.size()>0){
    return true;
  }
  return false;
}

bool DWAPlanner::isPathFeasible(const std::vector<Pose2D>& path)
{
  // Number of poses along the path that should be verified
  int look_ahead_idx = (int)path.size() - 1;

  for (int i = 0; i <= look_ahead_idx; ++i)
  {
    if (costmap_model_->footprintCost(path[i].x, path[i].y, path[i].theta, footprint_spec_, inscribed_radius_,
                                      circumscribed_radius_) == -1)
    {
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than
    // the specified threshold and interpolates in that case. (if obstacles are pushing two consecutive poses away, the
    // center between two consecutive poses might coincide with the obstacle ;-)!
    if (i < look_ahead_idx)
    {
      double delta_rot = NormalizeAngle(path[i + 1].theta - path[i].theta);
      Eigen::Vector2d delta_dist(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
      if (fabs(delta_rot) > M_PI || delta_dist.norm() > inscribed_radius_)
      {
        int n_additional_samples =
            std::max(std::ceil(fabs(delta_rot) / M_PI), std::ceil(delta_dist.norm() / inscribed_radius_)) - 1;

        Eigen::Vector2d intermediate_position(path[i].x, path[i].y);
        double intermediate_theta = path[i].theta;
        for (int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_position = intermediate_position + delta_dist / (n_additional_samples + 1.0);
          intermediate_theta = NormalizeAngle(intermediate_theta + delta_rot / (n_additional_samples + 1.0));
          if (costmap_model_->footprintCost(intermediate_position.x(), intermediate_position.y(), intermediate_theta,
                                            footprint_spec_, inscribed_radius_, circumscribed_radius_) == -1)
          {
            return false;
          }
        }
      }
    }
  }

  return true;
}

void DWAPlanner::publishCandidatePaths(const std::vector<std::vector<Pose2D> >& candidate_paths)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_->map_frame;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)candidate_paths.size(); i++)
  {
    for (int j = 0; j < (int)candidate_paths[i].size(); j++)
    {
      pose.pose.position.x = candidate_paths[i][j].x;
      pose.pose.position.y = candidate_paths[i][j].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(candidate_paths[i][j].theta);
      gui_path.poses.push_back(pose);
    }

    for (int j = (int)candidate_paths[i].size() - 1; j >= 0; j--)
    {
      pose.pose.position.x = candidate_paths[i][j].x;
      pose.pose.position.y = candidate_paths[i][j].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(candidate_paths[i][j].theta);
      gui_path.poses.push_back(pose);
    }
  }

  candidate_paths_pub_.publish(gui_path);
}

void DWAPlanner::publishFinalTraj(Trajectory& traj){
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_->map_frame;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  for(auto p:traj.getPoses()){
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
    gui_path.poses.push_back(pose);    
  }
  following_traj_pub_.publish(gui_path);
}


}  // namespace dwa_planner