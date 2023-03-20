#include "participant_game_lattice/participant_game_lattice.h"
#include <Eigen/Core>
#include <chrono>

namespace participant_game_lattice
{
ParticipantGameLattice::ParticipantGameLattice(base_local_planner::CostmapModel* costmap_model,
                                       const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                                       double circumscribed_radius, const BezierConfig& cfg, ros::NodeHandle nh)
  : costmap_model_(costmap_model)
  , footprint_spec_(footprint_spec)
  , inscribed_radius_(inscribed_radius)
  , circumscribed_radius_(circumscribed_radius)
  , cfg_(&cfg)
  , dt_(0.2)
{
  feasible_candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("feasible_candidate_paths", 1);
  feasible_backup_candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("feasible_backup_candidate_paths", 1);
  all_candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("all_candidate_paths", 1);
  tracked_path_pub_ = nh.advertise<nav_msgs::Path>("tracked_path", 1);
  sub_person = nh.subscribe<pedsim_msgs::TrackedPersons>("/pedsim_visualizer/tracked_persons", 1, boost::bind(&ParticipantGameLattice::PersonCallback, this ,_1));
}

ParticipantGameLattice::~ParticipantGameLattice()
{
}

/**
 * assgin for abs_candidate_targets_
 * */
void ParticipantGameLattice::initializeCandidateTargets(const double& forward_detect_latitude, std::vector<Pose2D>& abs_candidate_targets)
{
  const double forward_detect_longitude = 3;
  const double detect_number = 20;
  abs_candidate_targets.clear();
  forward_detect_latitude_ = forward_detect_latitude;
  Pose2D pose;
  pose.x = forward_detect_latitude;
  pose.y = -forward_detect_longitude/2;
  pose.theta = 0;
  double dy = forward_detect_longitude/detect_number;
  for (int i = 0; i < detect_number; i++)
  {
    abs_candidate_targets.push_back(pose);
    pose.y += dy;
  }
}

/**
 * assgin for candidate_targets_
 * */
void ParticipantGameLattice::transformCandidateTargets(const Pose2D& robot_pose, const std::vector<Pose2D>& global_plan){
  Pose2D pose;
  const double &x = robot_pose.x;
  const double &y = robot_pose.y;
  const double &theta = atan2(global_plan.back().y-y, global_plan.back().x-x);
  candidate_targets_.clear();
  candidate_backup_targets_.clear();
  for (auto target:abs_candidate_targets_){
    pose.x = target.x*cos(theta) - target.y*sin(theta) + x;
    pose.y = target.x*sin(theta) + target.y*cos(theta) + y;
    pose.theta = theta;
    candidate_targets_.push_back(pose);
  }
  for (auto target:abs_candidate_targets_){
    pose.x = -target.x*cos(theta) - target.y*sin(theta) + x;
    pose.y = -target.x*sin(theta) + target.y*cos(theta) + y;
    pose.theta = constrainTheta(theta+M_PI);
    candidate_backup_targets_.push_back(pose);
  }
}

/**
 * assgin for candidate_targets_
 * */
void ParticipantGameLattice::transformCandidateTargets(const Pose2D& robot_pose, const Pose2D& person_pose, const std::vector<Pose2D>& abs_candidate_targets, std::vector<Pose2D>& candidate_targets){
  Pose2D pose;
  const double &x = person_pose.x;
  const double &y = person_pose.y;
  const double theta = atan2(robot_pose.y-person_pose.y,robot_pose.x-person_pose.x);
  for (auto target:abs_candidate_targets){
    pose.x = target.x*cos(theta) - target.y*sin(theta) + x;
    pose.y = target.x*sin(theta) + target.y*cos(theta) + y;
    pose.theta = theta;
    candidate_targets.push_back(pose);
  }
}

/**
 * assgin for tracked_path_
 * */
void ParticipantGameLattice::scorePaths(const int& size_x, const int& size_y, const double& resolution, const double& origin_x, const double& origin_y, const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap){
  tracked_path_.clear();
  // robot can find a feasible path, go forward. 
  if (!feasible_candidate_paths_.empty()){
    double min_cost = 999999;
    for (auto path:feasible_candidate_paths_){
      if (path.empty()){
        continue;
      }
      double score = scorePath(path, size_x, size_y, resolution, origin_x, origin_y, global_plan, costmap);
      if (score<min_cost){
        min_cost=score;
        tracked_path_ = path;
      }
    }
  }
  // robot can't find a feasible path, it need to back up to compromise with people. 
  // use participant game
  else{
    if (predicted_trajectories_.empty()){
      return;
    }
    Pose2D nearest_person = predicted_trajectories_[0].front();
    for (auto traj:predicted_trajectories_){
      if (hypot(robot_pose_.x-traj.front().x, robot_pose_.y-traj.front().y) < hypot(robot_pose_.x-nearest_person.x, robot_pose_.y-nearest_person.y)){
        nearest_person = traj.front();
      }
    }
    // find the best path for the nearest person
    std::vector<Pose2D> person_abs_candidate_targets;
    initializeCandidateTargets(forward_detect_latitude_, person_abs_candidate_targets);
    std::vector<Pose2D> person_candidate_targets;
    transformCandidateTargets(robot_pose_, nearest_person, person_abs_candidate_targets, person_candidate_targets);
    std::vector<std::vector<Pose2D>> person_feasible_candidate_paths;
    for (auto target:person_candidate_targets){
      std::vector<Pose2D> temp_path = fitPath(nearest_person,target,50.0);
      if (isPathFeasible(temp_path)){
        person_feasible_candidate_paths.push_back(temp_path);
      }
    }
    std::vector<Pose2D> person_path;
    double min_cost = 999999;
    for (auto path:person_feasible_candidate_paths){
      double score = scorePath(path, size_x, size_y, resolution, origin_x, origin_y, costmap);
      if (score<min_cost){
        min_cost=score;
        person_path = path;
      }
    }
    // find the best backup path for the robot
    double max_score = -999;
    double score;
    for (auto robot_path:feasible_backup_candidate_paths_){
      double score1 = -0.01*scorePath(robot_path, size_x, size_y, resolution, origin_x, origin_y, costmap);
      double score2 = 99999;
      for (int i=0; i< std::min(int(person_path.size()),int(robot_path.size())); i++){
        if (hypot(person_path[i].x-robot_path[i].x,person_path[i].y-robot_path[i].y)<score2){
          score2 = std::min(score2, hypot(person_path[i].x-robot_path[i].x,person_path[i].y-robot_path[i].y));
        }
      }
      if (score1+score2 > max_score){
        max_score = score1+score2;
        tracked_path_ = robot_path;
      }
    }
  }
}

/**
 * score for a single path
 * */
double ParticipantGameLattice::scorePath(const std::vector<Pose2D>& path, const int& size_x, const int& size_y, const double& resolution, const double& origin_x, const double& origin_y, const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap){
  // cost1: occupy
  int mx,my;
  int occupy = 0;
  for(std::vector<Pose2D>::const_iterator it=path.begin(); it!=path.end(); it++){
    wolrd2map(it->x, it->y, mx, my, resolution, origin_x, origin_y);
    if (mx<0 || my<0 || mx>=size_x || my>=size_y){
      occupy = 256;
    }
    else{
      occupy = std::max(costmap[mx][my]-0,occupy);
    }
  }
  // cost2: distance to local target pose
  Pose2D endpose = path.back();
  Pose2D local_endpose = global_plan.back();
  double dis2end = sqrt(pow(endpose.x-local_endpose.x,2)+pow(endpose.y-local_endpose.y,2));
  // cost3: distance to reference path
  double dis2path = 1e10;
  for(std::vector<Pose2D>::const_iterator it=global_plan.begin(); it!=global_plan.end(); it++){
    dis2path = std::min(std::sqrt(std::pow(endpose.x-it->x,2)+std::pow(endpose.y-it->y,2)),dis2path);
  }
  return cfg_->occdist_scale*occupy + cfg_->goal_distance_bias*dis2end + cfg_->path_distance_bias*dis2path;
}

double ParticipantGameLattice::scorePath(const std::vector<Pose2D>& path, const int& size_x, const int& size_y, const double& resolution, const double& origin_x, const double& origin_y, unsigned char const* const* costmap){
  // cost1: occupy
  int mx,my;
  int occupy = 0;
  for(std::vector<Pose2D>::const_iterator it=path.begin(); it!=path.end(); it++){
    wolrd2map(it->x, it->y, mx, my, resolution, origin_x, origin_y);
    if (mx<0 || my<0 || mx>=size_x || my>=size_y){
      occupy = 256;
    }
    else{
      occupy = std::max(costmap[mx][my]-0,occupy);
    }
  }
  return occupy;
}



void ParticipantGameLattice::wolrd2map(const double& wx, const double& wy, int& mx, int& my, const double& resolution, const double& origin_x, const double& origin_y){
  mx = int((wx-origin_x)/resolution);
  my = int((wy-origin_y)/resolution);
}

void ParticipantGameLattice::fitAllPaths(const Pose2D& robot_pose){
  feasible_candidate_paths_.clear();
  const double disperse_number = 30;
  for (auto target:candidate_targets_){
    std::vector<Pose2D> temp_path = fitPath(robot_pose,target);
    if (isPathFeasible(temp_path) && (!isCollisionWithPerons(temp_path))){
    // if (isPathFeasible(temp_path)){
      feasible_candidate_paths_.push_back(temp_path);
    }
  }
  //back up the robot
  if (feasible_candidate_paths_.empty()){
    feasible_backup_candidate_paths_.clear();
    for (auto target:candidate_backup_targets_){
      std::vector<Pose2D> temp_path = fitPath(robot_pose,target,50.0);
      if (isPathFeasible(temp_path)){
        feasible_backup_candidate_paths_.push_back(temp_path);
      }
    }  
  }
}

void ParticipantGameLattice::PersonCallback(const pedsim_msgs::TrackedPersons::ConstPtr& persons) {
  // predict with constant velocity
  predicted_trajectories_.clear();
  for (auto person:persons->tracks){
    double temp_px = person.pose.pose.position.x;
    double temp_py = person.pose.pose.position.y;
    double temp_vx = person.twist.twist.linear.x;
    double temp_vy = person.twist.twist.linear.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(person.pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    std::vector<Pose2D> temp_trajectory;
    if (hypot(temp_vx,temp_vy)<0.05){
      temp_trajectory.push_back(Pose2D(temp_px,temp_py,yaw));
    }
    else{
      for (int t=0; t<=20; t++){
        temp_trajectory.push_back(Pose2D(temp_px+temp_vx*t*dt_, temp_py+temp_vy*t*dt_, yaw));
      }
    }
    
    predicted_trajectories_.push_back(temp_trajectory);
  }
}

bool ParticipantGameLattice::isCollisionWithPerons(const std::vector<Pose2D>& path){
  int min_size = path.size();
  double safe_dis = 0.25+0.35; //todo 改成从param接收
  for (auto person_trajectory:predicted_trajectories_){
    min_size = std::min(min_size,int(person_trajectory.size()));
  }
  bool is_collision = false;
  for (int i=0; i<min_size; i++){
    for (auto person_trajectory:predicted_trajectories_){
      if (hypot(person_trajectory[i].x-path[i].x,person_trajectory[i].y-path[i].y) < safe_dis){
        is_collision = true;
      }
    }
  }
  return is_collision;
}


double ParticipantGameLattice::constrainTheta(double theta){
  while (theta > M_PI){
    theta -= 2*M_PI;
  }
  while (theta < -M_PI){
    theta += 2*M_PI;
  }
  return theta;
}

/**
 * given start state and target state, fit the path with cubic Bezier 
 * */
std::vector<Pose2D> ParticipantGameLattice::fitPath(Pose2D p0, const Pose2D& p3){
  // const double disperse_number = std::min(100.0,fabs(forward_detect_latitude_/robot_vel_.v/dt_));

  double theta_r2t = constrainTheta(atan2(p3.y-p0.y, p3.x-p0.x));
  double oppo_theta = constrainTheta(p0.theta+M_PI);
  if (fabs(constrainTheta(theta_r2t - p0.theta)) > fabs(constrainTheta(theta_r2t - oppo_theta))){
    p0 = Pose2D(p0.x,p0.y,oppo_theta);
  }

  const double disperse_number = 30.0;
  double dis = 1.0/3.0*sqrt(pow(p0.x-p3.x,2)+pow(p0.y-p3.y,2));
  Pose2D p1_temp = Pose2D(p0.x+dis*cos(p0.theta),p0.y+dis*sin(p0.theta),0);
  Pose2D p2_temp = Pose2D(p3.x-dis*cos(p3.theta),p3.y-dis*sin(p3.theta),0);
  std::vector<Pose2D> temp_path; 
  for (double t=0.0; t<=disperse_number; t++){
    double x = CubicBezierPoint(p0.x,p1_temp.x,p2_temp.x,p3.x,t/disperse_number);
    double y = CubicBezierPoint(p0.y,p1_temp.y,p2_temp.y,p3.y,t/disperse_number);
    if (t==0)
      temp_path.push_back(Pose2D(p0.x,p0.y,p0.theta));
    else if (t==disperse_number)
      temp_path.push_back(Pose2D(p3.x,p3.y,p3.theta));
    else
      temp_path.push_back(Pose2D(x,y,0));
  }
  return temp_path;
}

std::vector<Pose2D> ParticipantGameLattice::fitPath(Pose2D p0, const Pose2D& p3, const double disperse_number){
  double theta_r2t = constrainTheta(atan2(p3.y-p0.y, p3.x-p0.x));
  double oppo_theta = constrainTheta(p0.theta+M_PI);
  if (fabs(constrainTheta(theta_r2t - p0.theta)) > fabs(constrainTheta(theta_r2t - oppo_theta))){
    p0 = Pose2D(p0.x,p0.y,oppo_theta);
  }
  
  double dis = 1.0/3.0*sqrt(pow(p0.x-p3.x,2)+pow(p0.y-p3.y,2));
  Pose2D p1_temp = Pose2D(p0.x+dis*cos(p0.theta),p0.y+dis*sin(p0.theta),0);
  Pose2D p2_temp = Pose2D(p3.x-dis*cos(p3.theta),p3.y-dis*sin(p3.theta),0);
  std::vector<Pose2D> temp_path; 
  for (double t=0.0; t<=disperse_number; t++){
    double x = CubicBezierPoint(p0.x,p1_temp.x,p2_temp.x,p3.x,t/disperse_number);
    double y = CubicBezierPoint(p0.y,p1_temp.y,p2_temp.y,p3.y,t/disperse_number);
    if (t==0)
      temp_path.push_back(Pose2D(p0.x,p0.y,p0.theta));
    else if (t==disperse_number)
      temp_path.push_back(Pose2D(p3.x,p3.y,p3.theta));
    else
      temp_path.push_back(Pose2D(x,y,0));
  }
  return temp_path;
}

Velocity ParticipantGameLattice::generateVel(const Pose2D& robot_pose){
  // find the state to be traced, just like fitPath()
  Pose2D p0 = tracked_path_.front();
  Pose2D p3 = tracked_path_.back();
  Pose2D p1 = tracked_path_[int(tracked_path_.size()/3.0)];
  Pose2D p2 = tracked_path_[int(tracked_path_.size()/3.0*2.0)];
  double x = p1.x;
  double y = p1.y;

  // double dis = 1.0/3.0*hypot(p0.x-p3.x, p0.y-p3.y);
  // Pose2D p1_temp = Pose2D(p0.x+dis*cos(p0.theta),p0.y+dis*sin(p0.theta),0);
  // Pose2D p2_temp = Pose2D(p3.x-dis*cos(p3.theta),p3.y-dis*sin(p3.theta),0);
  // double x = CubicBezierPoint(p0.x,p1_temp.x,p2_temp.x,p3.x,0.3);
  // double y = CubicBezierPoint(p0.y,p1_temp.y,p2_temp.y,p3.y,0.3);
  
  // the related state
  double delta_x = x-p0.x;
  double delta_y = y-p0.y;
  double delta_theta = constrainTheta(atan2(delta_y,delta_x)-robot_pose.theta);
  // case1: if the robot is near the target pose, stop
  if (hypot(p0.x-p3.x,p0.y-p3.y) < cfg_->xy_goal_tolerance){
    return Velocity(0,0);
  }
  // case2: trace the target point with P controller
  double rho = hypot(delta_x, delta_y);
  double alpha = delta_theta;

  // allow the robot to go oppositely
  if(fabs(alpha) > M_PI_2)
  {
    rho = -1* rho;
    if(alpha > 0)
      alpha = -M_PI + alpha;

    else
      alpha = M_PI + alpha;
  }
  double k_rho = 2.0;
  double k_alpha = 2.2;
  double desired_v = k_rho * rho;
  double desired_w = k_alpha * alpha;
  double v,w;
  // constrain by acceleration
  if (desired_v > robot_vel_.v){
    v = std::min(robot_vel_.v + cfg_->acc_lim_x*cfg_->control_period, desired_v);
  }
  else{
    v = std::max(robot_vel_.v - cfg_->acc_lim_x*cfg_->control_period, desired_v);
  }

  if (desired_w > robot_vel_.omega){
    w = std::min(robot_vel_.omega + cfg_->acc_lim_theta*cfg_->control_period, desired_w);
  }
  else{
    w = std::max(robot_vel_.omega - cfg_->acc_lim_theta*cfg_->control_period, desired_w);
  }
  // constrain by velocity
  if(v > cfg_->max_vel_x)
    v = cfg_->max_vel_x;
  else if(v < -cfg_->max_vel_x)
    v = -cfg_->max_vel_x;

  if(w > cfg_->max_vel_theta)
    w = cfg_->max_vel_theta;
  else if(w < -cfg_->max_vel_theta)
    w = -cfg_->max_vel_theta;
  return Velocity(v,w);
}


bool ParticipantGameLattice::computeVelocityCommands(const Velocity& robot_vel, const Pose2D& robot_pose,
                                                 const std::vector<Pose2D>& global_plan,
                                                 unsigned char const* const* costmap, int size_x, int size_y,
                                                 double resolution, double origin_x, double origin_y, Velocity& cmd_vel)
{
  robot_vel_ = robot_vel;
  robot_pose_ = robot_pose;
  double dis2target = hypot(robot_pose.x-global_plan.back().x,robot_pose.y-global_plan.back().y);
  initializeCandidateTargets(std::min(dis2target,cfg_->max_global_plan_lookahead_dist), abs_candidate_targets_);
  transformCandidateTargets(robot_pose, global_plan);
  fitAllPaths(robot_pose);
  scorePaths(size_x, size_y, resolution, origin_x, origin_y, global_plan, costmap);
  if (tracked_path_.empty()){
    cmd_vel = Velocity(0,0);
    return false;
  }
  cmd_vel = generateVel(robot_pose);
  publishBezierPath(tracked_path_, tracked_path_pub_);
  publishBezierPaths(feasible_candidate_paths_,feasible_candidate_paths_pub_);
  publishBezierPaths(feasible_backup_candidate_paths_,feasible_backup_candidate_paths_pub_);
  return true;
}

bool ParticipantGameLattice::isPathFeasible(const std::vector<Pose2D>& path)
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

void ParticipantGameLattice::publishBezierPaths(const std::vector<std::vector<Pose2D> >& bezier_paths,
                                            ros::Publisher publisher)
{
  if (bezier_paths.empty())
    return;

  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_->map_frame;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)bezier_paths.size(); i++)
  {
    for (int j = 0; j < (int)bezier_paths[i].size(); j++)
    {
      pose.pose.position.x = bezier_paths[i][j].x;
      pose.pose.position.y = bezier_paths[i][j].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(bezier_paths[i][j].theta);
      gui_path.poses.push_back(pose);
    }

    for (int j = (int)bezier_paths[i].size() - 1; j >= 0; j--)
    {
      pose.pose.position.x = bezier_paths[i][j].x;
      pose.pose.position.y = bezier_paths[i][j].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(bezier_paths[i][j].theta);
      gui_path.poses.push_back(pose);
    }
  }

  publisher.publish(gui_path);
}

void ParticipantGameLattice::publishBezierPath(const std::vector<Pose2D>& bezier_path, ros::Publisher publisher)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_->map_frame;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)bezier_path.size(); i++)
  {
    pose.pose.position.x = bezier_path[i].x;
    pose.pose.position.y = bezier_path[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(bezier_path[i].theta);
    gui_path.poses.push_back(pose);
  }

  publisher.publish(gui_path);
}

}  // namespace participant_game_lattice