/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <QApplication>
#include <algorithm>

#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/simulator.h>

#include <pedsim_utils/geometry.h>

using namespace pedsim;

Simulator::Simulator(const ros::NodeHandle& node) : nh_(node) {
  dynamic_reconfigure::Server<SimConfig>::CallbackType f;
  f = boost::bind(&Simulator::reconfigureCB, this, _1, _2);
  server_.setCallback(f);
}

Simulator::~Simulator() {
  // shutdown service servers and publishers
  pub_obstacles_.shutdown();
  pub_agent_states_.shutdown();
  pub_agent_groups_.shutdown();
  pub_robot_position_.shutdown();
  pub_waypoints_.shutdown();
  pub_voronoi_grid_.shutdown();
  pub_passable_map_.shutdown();
  pub_voronoi_edge_.shutdown();
  pub_voronoi_node_.shutdown();

  delete robot_;
  QCoreApplication::exit(0);
}

bool Simulator::initializeSimulation() {
  int queue_size = 0;
  nh_.param<int>("default_queue_size", queue_size, 1);
  ROS_INFO_STREAM("Using default queue size of "
                  << queue_size << " for publisher queues... "
                  << (queue_size == 0
                          ? "NOTE: This means the queues are of infinite size!"
                          : ""));
  // setup ros publishers
  pub_obstacles_ = nh_.advertise<pedsim_msgs::LineObstacles>("simulated_walls", queue_size);
  pub_agent_states_ = nh_.advertise<pedsim_msgs::AgentStates>("simulated_agents", queue_size);
  pub_agent_groups_ = nh_.advertise<pedsim_msgs::AgentGroups>("simulated_groups", queue_size);
  pub_robot_position_ = nh_.advertise<nav_msgs::Odometry>("/odom", queue_size);
  pub_waypoints_ = nh_.advertise<pedsim_msgs::Waypoints>("simulated_waypoints", queue_size);
  pub_voronoi_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);
  pub_passable_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("passable_map", 1);
  pub_voronoi_node_ = nh_.advertise<visualization_msgs::MarkerArray>("voronoi_node", 1);
  pub_voronoi_edge_ = nh_.advertise<visualization_msgs::MarkerArray>("voronoi_edge", 1);
  sub_costmap_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map",100, boost::bind(&Simulator::costmapUpdateCallback, this ,_1));

  // setup TF listener and other pointers
  transform_listener_.reset(new tf::TransformListener());
  robot_ = nullptr;
  last_theta_ = -10;

  // load additional parameters
  std::string scene_file_param;
  nh_.param<std::string>("scene_file", scene_file_param, "");
  if (scene_file_param == "") {
    ROS_ERROR_STREAM("Invalid scene file: " << scene_file_param);
    return false;
  }

  ROS_INFO_STREAM("Loading scene [" << scene_file_param << "] for simulation");

  const QString scenefile = QString::fromStdString(scene_file_param);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false) {
    ROS_ERROR_STREAM(
        "Could not load the scene file, please check the paths and param "
        "names : "
        << scene_file_param);
    return false;
  }

  nh_.param<bool>("enable_groups", CONFIG.groups_enabled, true);
  nh_.param<double>("max_robot_speed", CONFIG.max_robot_speed, 1.5);
  nh_.param<double>("update_rate", CONFIG.updateRate, 25.0);
  nh_.param<double>("simulation_factor", CONFIG.simulationFactor, 1.0);

  int robot_mode = 1;
  nh_.param<int>("robot_mode", robot_mode, 2);
  CONFIG.robot_mode = static_cast<RobotMode>(robot_mode);
  int person_mode = 1;
  nh_.param<int>("person_mode", person_mode, 2);
  CONFIG.person_mode = static_cast<PersonMode>(person_mode);

  double spawn_period;
  nh_.param<double>("spawn_period", spawn_period, 5.0);
  nh_.param<std::string>("frame_id", frame_id_, "odom");
  nh_.param<std::string>("robot_base_frame_id", robot_base_frame_id_,
      "base_link");

  spawn_timer_ =
      nh_.createTimer(ros::Duration(spawn_period), &Simulator::spawnCallback, this);
  
  return true;
}

void Simulator::runSimulation() {
  ros::Rate r(CONFIG.updateRate);

  while (ros::ok()) {
    if (!robot_) {
      // setup the robot
      for (Agent* agent : SCENE.getAgents()) {
        if (agent->getType() == Ped::Tagent::ROBOT) {
          robot_ = agent;
        }
        else{
          agent->setSubscriber(nh_);
          agent->setPublisher(nh_);          
        }
      }
    }


    publishGaze();    
    updateRobotPositionFromTF();
    SCENE.moveAllAgents(&dg_);
    publishAgents();
    publishGroups();
    publishRobotPosition();
    publishObstacles();
    publishWaypoints();
    publishVoronoiGrid();
    publishPassableMap();
    pubNode();
    pubEdge();

    ros::spinOnce();
    r.sleep();
  }
}

void Simulator::reconfigureCB(pedsim_simulator::PedsimSimulatorConfig& config, uint32_t level) {
  CONFIG.updateRate = config.update_rate;
  CONFIG.simulationFactor = config.simulation_factor;

  // update force scaling factors
  CONFIG.setObstacleForce(config.force_obstacle);
  CONFIG.setObstacleSigma(config.sigma_obstacle);
  CONFIG.setSocialForce(config.force_social);
  CONFIG.setVoronoiForce(config.force_voronoi);
  CONFIG.setGroupGazeForce(config.force_group_gaze);
  CONFIG.setGroupCoherenceForce(config.force_group_coherence);
  CONFIG.setGroupRepulsionForce(config.force_group_repulsion);
  CONFIG.setRandomForce(config.force_random);
  CONFIG.setAlongWallForce(config.force_wall);

  ROS_INFO_STREAM("Updated sim with live config: Rate=" << CONFIG.updateRate
                                                        << " incoming rate="
                                                        << config.update_rate);
}

void Simulator::spawnCallback(const ros::TimerEvent& event) {
  ROS_DEBUG_STREAM("Spawning new agents.");

  for (const auto& sa : SCENE.getSpawnAreas()) {
    AgentCluster* agentCluster = new AgentCluster(sa->x, sa->y, sa->n);
    agentCluster->setDistribution(sa->dx, sa->dy);
    agentCluster->setType(static_cast<Ped::Tagent::AgentType>(0));

    for (const auto& wp_name : sa->waypoints) {
      agentCluster->addWaypoint(SCENE.getWaypointByName(wp_name));
    }

    SCENE.addAgentCluster(agentCluster);
  }
}

void Simulator::costmapUpdateCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  if (map_without_people_.data.empty()){
    map_without_people_ = *msg;
    // ros::Time t = ros::Time::now();

    // receives the map information from topic
    int nx_ = msg->info.width;
    int ny_ = msg->info.height;
    int map_size = msg->data.size();
    auto map_char = msg->data;

    // construct a 2D map. true represents the point is obstacle-occupied
    bool **map_bool = NULL;
    map_bool = new bool*[nx_];
    for (int x=0; x<nx_; x++) {
        map_bool[x] = new bool[ny_];
    }
    for (int i=0; i<map_size; i++){
        if ((int)map_char[i] != 100){
            map_bool[i%nx_][i/nx_] = false;
        }
        else{
            map_bool[i%nx_][i/nx_] = true;
        }
    }
    // initialize voronoi object it with the map
    voronoi_.initializeMap(nx_, ny_, map_bool);
    // update distance map and Voronoi diagram
    voronoi_.update(); 
    // prune the Voronoi
    voronoi_.prune();
    


    for (int x=0; x<nx_; x++) {
        delete []map_bool[x];
    }
    delete []map_bool;
    
    // voronoi_map保存所有剔除边界点后的voronoi点
    std::vector<std::vector<bool>> voronoi_map(nx_, std::vector<bool>(ny_, false));
    std::vector<std::vector<float>> distance_map(nx_, std::vector<float>(ny_, 999.9));
    std::vector<std::vector<bool>> passable_map(nx_, std::vector<bool>(ny_, false));
    std::vector<std::vector<bool>> passable_map2(nx_, std::vector<bool>(ny_, false));

    for (int x=0; x<nx_; ++x){
        for (int y=0; y<ny_; ++y){
          distance_map[x][y] = voronoi_.getDistance(x,y);
          if (voronoi_.getDistance(x,y)>0.3/0.1 && !(x==0 || x==nx_-1 || y==0 || y==ny_-1)){  // 设人的半径为0.3
            if (voronoi_.isVoronoi(x,y)){
              voronoi_map[x][y] = true;              
            }
          }
          if (voronoi_.getDistance(x,y)>0.3/0.1 && !(x==0 || x==nx_-1 || y==0 || y==ny_-1)){  // 设人的半径为0.3
            passable_map[x][y] = true;           
          }
        }
    }
    
    std::vector<std::pair<int,int>> iter = {{-1,-1},{-1,1},{1,-1},{1,1}};
    for (int x=0; x<nx_; ++x){
      for (int y=0; y<ny_; ++y){
        int passable_count = 0;
        int obstacle_count = 0;      
        if (passable_map[x][y]){
          passable_map2[x][y] = true;
        }
        else if (x==0 || x==nx_-1 || y==0 || y==ny_-1){
          passable_map2[x][y] = false;
        }
        else{
          for (auto it:iter){
            if (passable_map[x+it.first][y+it.second])
              ++passable_count;
            else
              ++obstacle_count;
          }
          if (passable_count==3 && obstacle_count==1){
            passable_map2[x][y] = true;
          }
          else{
            passable_map2[x][y] = false;
          }
        }
      }
    }
    voronoi_map_ = voronoi_map;
    passable_map_ = passable_map2;
    dg_.constructDecisionGraph(voronoi_map, distance_map, passable_map, *msg);
    // ROS_INFO("Time: %f sec", (ros::Time::now() - t).toSec());
  }
}

void Simulator::updateRobotPositionFromTF() {
  if (!robot_ || CONFIG.robot_mode==RobotMode::SOCIAL_DRIVE) return;
  // 如果通过外部控制，就直接监听TF变换就行了
  if (CONFIG.robot_mode == RobotMode::TELEOPERATION ||
      CONFIG.robot_mode == RobotMode::CONTROLLED) {

    // Get robot position via TF
    tf::StampedTransform tfTransform;
    try {
      transform_listener_->lookupTransform(frame_id_, robot_base_frame_id_,
                                           ros::Time(0), tfTransform);
    } catch (tf::TransformException& e) {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "TF lookup from " << robot_base_frame_id_ << " to " << frame_id_
          << " failed. Reason: " << e.what());
      return;
    }

    const double x = tfTransform.getOrigin().x();
    const double y = tfTransform.getOrigin().y();
    const double dx = x - last_robot_pose_.getOrigin().x(),
                 dy = y - last_robot_pose_.getOrigin().y();
    const double dt =
        tfTransform.stamp_.toSec() - last_robot_pose_.stamp_.toSec();
    double vx = dx / dt, vy = dy / dt;

    double theta = atan2(vy,vx);
    double omega = (theta-robot_->gettheta())/dt;
    if (!std::isfinite(vx)) vx = 0;
    if (!std::isfinite(vy)) vy = 0;
    if (!std::isfinite(omega)) omega = 0;

    ROS_DEBUG_STREAM("rx, ry: " << robot_->getx() << ", " << robot_->gety() << " vs: " << x << ", " << y);

    robot_->setX(x);
    robot_->setY(y);
    robot_->settheta(theta);
    robot_->setvx(vx);
    robot_->setvy(vy);
    robot_->setomega(omega);


    ROS_DEBUG_STREAM("Robot speed: " << std::hypot(vx, vy) << " dt: " << dt);

    last_robot_pose_ = tfTransform;
    last_robot_orientation_ = poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
  }
}

void Simulator::publishRobotPosition() {
  if (robot_ == nullptr) return;
  if (last_theta_== -10)
    last_theta_ = std::atan2(robot_->getvy(), robot_->getvx());

  nav_msgs::Odometry robot_location;
  robot_location.header = createMsgHeader();
  robot_location.child_frame_id = robot_base_frame_id_;

  robot_location.pose.pose.position.x = robot_->getx();
  robot_location.pose.pose.position.y = robot_->gety();
  double current_theta = std::atan2(robot_->getvy(), robot_->getvx());
  double delta_theta = (current_theta-last_theta_);
  while (delta_theta>M_PI)
    delta_theta -= 2*M_PI;
  while (delta_theta<-M_PI)
    delta_theta += 2*M_PI;


  if (fabs(delta_theta)<M_PI*0.5){
    robot_location.pose.pose.orientation = angleToQuaternion(current_theta);
    last_robot_orientation_ = robot_location.pose.pose.orientation;
    robot_location.twist.twist.linear.x = sign(robot_location.twist.twist.linear.x) * hypot(robot_->getvx(), robot_->getvy());
    // robot_location.twist.twist.linear.x = hypot(robot_->getvx(), robot_->getvy());
    robot_location.twist.twist.angular.z = robot_->getomega();
  }
  else{
    current_theta = current_theta + M_PI;
    while (current_theta>M_PI)
      current_theta -= 2*M_PI;
    while (current_theta<-M_PI)
      current_theta += 2*M_PI;        
    robot_location.pose.pose.orientation = angleToQuaternion(current_theta);
    last_robot_orientation_ = robot_location.pose.pose.orientation;
    robot_location.twist.twist.linear.x = -sign(robot_location.twist.twist.linear.x) * hypot(robot_->getvx(), robot_->getvy());
    // robot_location.twist.twist.linear.x = - hypot(robot_->getvx(), robot_->getvy());
    robot_location.twist.twist.angular.z = robot_->getomega();    
  }   
  // }
  last_theta_ = current_theta;
  pub_robot_position_.publish(robot_location);
  if (CONFIG.robot_mode==RobotMode::SOCIAL_DRIVE){
    pubTfForSelfSFM();
  }
}

void Simulator::pubTfForSelfSFM() {
  static tf::Transform g_currentPose_for_SFM;
  static tf::TransformBroadcaster g_transformBroadcaster_for_SFM;
  // Update pose
  g_currentPose_for_SFM.getOrigin().setX(robot_->getx());
  g_currentPose_for_SFM.getOrigin().setY(robot_->gety());
  g_currentPose_for_SFM.setRotation(tf::createQuaternionFromRPY(0, 0, atan2(robot_->getvy(),robot_->getvx())));

  // Broadcast transform
  g_transformBroadcaster_for_SFM.sendTransform(tf::StampedTransform(
      g_currentPose_for_SFM, ros::Time::now(), frame_id_, robot_base_frame_id_));
}

void Simulator::publishAgents() {
  if (SCENE.getAgents().size() < 2) {
    return;
  }

  pedsim_msgs::AgentStates all_status;
  all_status.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector& v) {
    geometry_msgs::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent* a : SCENE.getAgents()) {
    pedsim_msgs::AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = a->getType();
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();
    auto theta = a->gettheta();
    state.pose.orientation = pedsim::angleToQuaternion(theta);

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);
    if (a->getType() == Ped::Tagent::ELDER) {
      state.social_state = pedsim_msgs::AgentState::TYPE_STANDING;
    }

    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT) {
      continue;
    }

    // Forces.
    pedsim_msgs::AgentForce agent_forces;
    // agent_forces.desired_force = VecToMsg(a->getDesiredDirection());
    agent_forces.desired_force = VecToMsg(a->getVoronoiForce());
    agent_forces.social_force = VecToMsg(a->getSocialForce());
    // agent_forces.obstacle_force = VecToMsg(a->getVoronoiForce() + a->getSocialForce());
    
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;

    all_status.agent_states.push_back(state);
  }

  pub_agent_states_.publish(all_status);
}

void Simulator::publishGroups() {
  if (!CONFIG.groups_enabled) {
    ROS_DEBUG_STREAM("Groups are disabled, no group data published: flag="
                     << CONFIG.groups_enabled);
    return;
  }

  if (SCENE.getGroups().size() < 1) {
    return;
  }

  pedsim_msgs::AgentGroups sim_groups;
  sim_groups.header = createMsgHeader();

  for (const auto& ped_group : SCENE.getGroups()) {
    if (ped_group->memberCount() <= 1) continue;

    pedsim_msgs::AgentGroup group;
    group.group_id = ped_group->getId();
    group.age = 10;
    const Ped::Tvector com = ped_group->getCenterOfMass();
    group.center_of_mass.position.x = com.x;
    group.center_of_mass.position.y = com.y;

    for (const auto& member : ped_group->getMembers()) {
      group.members.emplace_back(member->getId());
    }
    sim_groups.groups.emplace_back(group);
  }
  pub_agent_groups_.publish(sim_groups);
}

void Simulator::publishObstacles() {
  pedsim_msgs::LineObstacles sim_obstacles;
  sim_obstacles.header = createMsgHeader();
  for (const auto& obstacle : SCENE.getObstacles()) {
    pedsim_msgs::LineObstacle line_obstacle;
    line_obstacle.start.x = obstacle->getax();
    line_obstacle.start.y = obstacle->getay();
    line_obstacle.start.z = 0.0;
    line_obstacle.end.x = obstacle->getbx();
    line_obstacle.end.y = obstacle->getby();
    line_obstacle.end.z = 0.0;
    sim_obstacles.obstacles.push_back(line_obstacle);
  }
  pub_obstacles_.publish(sim_obstacles);
}

void Simulator::publishWaypoints() {
  pedsim_msgs::Waypoints sim_waypoints;
  sim_waypoints.header = createMsgHeader();
  for (const auto& waypoint : SCENE.getWaypoints()) {
    pedsim_msgs::Waypoint wp;
    wp.name = waypoint->getName().toStdString();
    wp.behavior = waypoint->getBehavior();
    wp.radius = waypoint->getRadius();
    wp.position.x = waypoint->getPosition().x;
    wp.position.y = waypoint->getPosition().y;
    sim_waypoints.waypoints.push_back(wp);
  }
  pub_waypoints_.publish(sim_waypoints);
}

void Simulator::publishGaze()
{
  visualization_msgs::Marker gazeVec;
  visualization_msgs::Marker gazePoint;

  int agent_id=0;

  for (const Agent* a : SCENE.getAgents()) 
  {
    if (a->getType()==Ped::Tagent::ROBOT){
      continue;
    }
    else{

      // publish the gaze vector
      gazeVec.header.frame_id="odom";
      gazeVec.header.stamp=ros::Time::now();
      gazeVec.ns="gaze_vector";
      gazeVec.id=agent_id;
      gazeVec.action =visualization_msgs::Marker::ADD;
      
      gazeVec.type = visualization_msgs::Marker::ARROW;

      gazeVec.pose.position.x=a->getPosition().x;
      gazeVec.pose.position.y=a->getPosition().y;
      gazeVec.pose.position.z=1.6;

      gazeVec.pose.orientation.x=a->getGazeOrientation().x;
      gazeVec.pose.orientation.y=a->getGazeOrientation().y;
      gazeVec.pose.orientation.z=a->getGazeOrientation().z;
      gazeVec.pose.orientation.w=a->getGazeOrientation().yaw;

      gazeVec.scale.x = 1;
      gazeVec.scale.y = 0.1;
      gazeVec.scale.z = 0.1;

      gazeVec.color.a = 1.0;
      gazeVec.color.r = 0.0;
      gazeVec.color.g = 1.0;
      gazeVec.color.b = 0.0;

      a->gazePublish(gazeVec);
    }
  }
}

void Simulator::publishVoronoiGrid()
{
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header = map_without_people_.header;
    grid.header.stamp = ros::Time::now();
    grid.info = map_without_people_.info;
    auto nx_ = grid.info.width;
    auto ny_ = grid.info.height;

    grid.data.resize(nx_ * ny_);

    for (unsigned int x = 0; x < nx_; x++)
    {
        for (unsigned int y = 0; y < ny_; y++)
        {
            if(voronoi_map_[x][y])
                grid.data[x + y*nx_] = (char)127;
            else
                grid.data[x + y*nx_] = (char)0;
        }
    }

    pub_voronoi_grid_.publish(grid);
}

void Simulator::publishPassableMap()
{
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header = map_without_people_.header;
    grid.header.stamp = ros::Time::now();
    grid.info = map_without_people_.info;
    auto nx_ = grid.info.width;
    auto ny_ = grid.info.height;

    grid.data.resize(nx_ * ny_);

    for (unsigned int x = 0; x < nx_; x++)
    {
        for (unsigned int y = 0; y < ny_; y++)
        {
            if(passable_map_[x][y])
                grid.data[x + y*nx_] = (char)27;
            else
                grid.data[x + y*nx_] = (char)0;
        }
    }
    pub_passable_map_.publish(grid);
}

void Simulator::pubNode(){
  auto node_list = dg_.graph_.getNodeList();
  visualization_msgs::MarkerArray markerArray;
  int i=0;
  double height = 0.1;
  for (auto node:node_list){
    float color[3] = {1,0,0};
    if (node.attribute_ == "dump")
      color[0]=0;
      // continue;
    // height += 0.1;
    visualization_msgs::Marker marker;
    double wx = map_without_people_.info.origin.position.x + node.x*map_without_people_.info.resolution + 0.5*map_without_people_.info.resolution;
    double wy = map_without_people_.info.origin.position.y + node.y*map_without_people_.info.resolution + 0.5*map_without_people_.info.resolution;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // marker.ns = "basic_shapes";
    marker.id = i++;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::POINTS;;  //points

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    geometry_msgs::Point poi;
    poi.x = wx;
    poi.y = wy;
    poi.z = height;
    marker.points.push_back(poi);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    markerArray.markers.push_back(marker);
  }
  pub_voronoi_node_.publish(markerArray);    
}

void Simulator::pubEdge(){
  auto graph = dg_.graph_;
  visualization_msgs::MarkerArray markerArray;
  int i=0;
  double height = 0.05;
  double width = 0.1;
  std::vector<float> color = {0,0,0};
  std::vector<float> color_avoiding = {0,0,1};
  std::vector<float> color_moving_single = {0,0.8,0};
  std::vector<float> color_moving_double = {0,0.5,0};
  for (int j=0; j<graph.getNodeList().size();j++){
    for (int k=j+1; k<graph.getNodeList().size(); k++){
      auto edge = graph.getEdge(j,k);
      auto path = edge.getPath();
      if (path.size()==0)
        continue;
      if (edge.getDirAttribute()=="backtoway" || edge.getDirAttribute()=="giveway"){
        color = color_avoiding;
      }
      if (edge.getDirAttribute()=="forward" || edge.getDirAttribute()=="backward"){
        if (edge.getWidthAttribute()=="single"){
          color = color_moving_single;
        }
        else{
          color = color_moving_double;
        }
      }
      for (int l=0; l<path.size()-1; l++){
        visualization_msgs::Marker marker;
        double wx1 = map_without_people_.info.origin.position.x + path[l].first*map_without_people_.info.resolution+0.5*map_without_people_.info.resolution;
        double wy1 = map_without_people_.info.origin.position.y + path[l].second*map_without_people_.info.resolution+0.5*map_without_people_.info.resolution;
        double wx2 = map_without_people_.info.origin.position.x + path[l+1].first*map_without_people_.info.resolution+0.5*map_without_people_.info.resolution;
        double wy2 = map_without_people_.info.origin.position.y + path[l+1].second*map_without_people_.info.resolution+0.5*map_without_people_.info.resolution;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
    
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // marker.ns = "basic_shapes";
        marker.id = i++;
    
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::LINE_STRIP;;  //points
    
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
    
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        geometry_msgs::Point poi;
        poi.x = wx1;
        poi.y = wy1;
        poi.z = height;
        marker.points.push_back(poi);
        poi.x = wx2;
        poi.y = wy2;
        poi.z = height;
        marker.points.push_back(poi);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = width;
        marker.scale.y = width;
        marker.scale.z = 0.1;
    
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        markerArray.markers.push_back(marker);
      }
    }
  }

  pub_voronoi_edge_.publish(markerArray);    
}

std::string Simulator::agentStateToActivity(
    const AgentStateMachine::AgentState& state) const {
  std::string activity = "Unknown";
  switch (state) {
    case AgentStateMachine::AgentState::StateWalking:
      activity = pedsim_msgs::AgentState::TYPE_INDIVIDUAL_MOVING;
      break;
    case AgentStateMachine::AgentState::StateGroupWalking:
      activity = pedsim_msgs::AgentState::TYPE_GROUP_MOVING;
      break;
    case AgentStateMachine::AgentState::StateQueueing:
      activity = pedsim_msgs::AgentState::TYPE_WAITING_IN_QUEUE;
      break;
    case AgentStateMachine::AgentState::StateShopping:
      break;
    case AgentStateMachine::AgentState::StateNone:
      break;
    case AgentStateMachine::AgentState::StateWaiting:
      break;
  }
  return activity;
}

std_msgs::Header Simulator::createMsgHeader() const {
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  return msg_header;
}

float Simulator::sign(float x){
  // todo 应该是个约束 很小的值内 sign保持不变？
  if (x>0)
    return 1;
  else if (x==0)
    return 1;
  else
    return -1;
}
