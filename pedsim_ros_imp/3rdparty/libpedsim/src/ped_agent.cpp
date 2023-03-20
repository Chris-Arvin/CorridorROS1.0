//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 20012 by Christian Gloor
//

#include "ped_agent.h"
#include "ped_obstacle.h"
#include "ped_scene.h"
#include "ped_waypoint.h"

#include <algorithm>
#include <cmath>
#include <random>

using namespace std;

default_random_engine generator;

/// Default Constructor
Ped::Tagent::Tagent() {
  static int staticid = 0;
  id = staticid++;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  v.x = 0;
  v.y = 0;
  v.z = 0;
  theta = M_PI;
  omega=0;
  type = ADULT;
  scene = nullptr;
  teleop = false;

  // assign random maximal speed in m/s
  normal_distribution<double> distribution(1.0, 0.26);
  // 在这里设置人的最大速度
  vmax = distribution(generator);
  vmax = 0.5;
  // vmax = 1.0;

  forceFactorDesired = 1.0;
  forceFactorSocial = 2.1;
  forceFactorVoronoi = 10.0;
  forceFactorObstacle = 10.0;
  forceSigmaObstacle = 0.8;

  agentRadius = 0.35;
  relaxationTime = 0.5;
  robotPosDiffScalingFactor = 2;


}

/// Destructor
Ped::Tagent::~Tagent() {}

/// Assigns a Tscene to the agent. Tagent uses this to iterate over all
/// obstacles and other agents in a scene.
/// The scene will invoke this function when Tscene::addAgent() is called.
/// \warning Bad things will happen if the agent is not assigned to a scene. But
/// usually, Tscene takes care of that.
/// \param   *s A valid Tscene initialized earlier.
void Ped::Tagent::assignScene(Ped::Tscene* sceneIn) { scene = sceneIn; }

void Ped::Tagent::removeAgentFromNeighbors(const Ped::Tagent* agentIn) {
  // search agent in neighbors, and remove him
  set<const Ped::Tagent*>::iterator foundNeighbor = neighbors.find(agentIn);
  if (foundNeighbor != neighbors.end()) neighbors.erase(foundNeighbor);
}

void Ped::Tagent::registerOnDG(DecisionGraph* dg){
  // std::cout<<"registerOnDG: "<<id<<": ("<<p.x<<","<<p.y<<") "<<desiredDirection.x<<","<<desiredDirection.y<<std::endl;
  dg->updatePeople(id, p.x, p.y, p.vx, p.vy, desiredDirection.x);
}

Ped::Tvector Ped::Tagent::voronoiForce(DecisionGraph* dg){
  auto point = dg->getDecision(id);
  return Ped::Tvector(point.first,point.second);
}


/// Sets the maximum velocity of an agent (vmax). Even if pushed by other
/// agents, it will not move faster than this.
/// \param   pvmax The maximum velocity. In scene units per timestep, multiplied
/// by the simulation's precision h.
void Ped::Tagent::setVmax(double pvmax) { vmax = pvmax; }

/// Defines how much the position difference between this agent
/// and a robot is scaled: the bigger the number is, the smaller
/// the position based force contribution will be.
/// \param   scalingFactor should be positive.
void Ped::Tagent::setRobotPosDiffScalingFactor(double scalingFactor) {
  if (scalingFactor > 0) {
    robotPosDiffScalingFactor = scalingFactor;
  }
}

/// Sets the agent's position. This, and other getters returning coordinates,
/// will eventually changed to returning a
/// Tvector.
/// \param   px Position x
/// \param   py Position y
/// \param   pz Position z
void Ped::Tagent::setPosition(double px, double py, double pz) {
  p.x = px;
  p.y = py;
  p.z = pz;
}

void Ped::Tagent::setGazeOrientation(double px, double py, double pz, double pw)
{
  gori.x=px;
  gori.y=py;
  gori.z=pz;
  gori.yaw=pw;
}

void Ped::Tagent::setGazeTarget(double px,double py,double pz,double pmode)
{
  gtar.x=px;
  gtar.y=py;
  gtar.z=pz;
  gtar.yaw=pmode;
}
/// Sets the factor by which the desired force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorDesired(double f) { forceFactorDesired = f; }

/// Sets the factor by which the social force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorSocial(double f) { forceFactorSocial = f; }

/// Sets the factor by which the obstacle force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorObstacle(double f) { forceFactorObstacle = f; }

void Ped::Tagent::setForceFactorVoronoi(double f) {forceFactorVoronoi = f;}

/// Calculates the force between this agent and the next assigned waypoint.
/// If the waypoint has been reached, the next waypoint in the list will be
/// selected.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::desiredForce() {
  // get destination
  Twaypoint* waypoint = getCurrentWaypoint();

  // if there is no destination, don't move
  if (waypoint == NULL) {
    desiredDirection = Ped::Tvector();
    Tvector antiMove = -v / relaxationTime;
    return antiMove;
  }

  // compute force
  Tvector force = waypoint->getForce(*this, &desiredDirection);

  return force;
}

/**
 * if (moving edge)
 *  for each person at the moving edge
 *    push laterally
 * else if (avoding edge)
 *  for each person at the avoding edge and behind the ego
 *    push lattitudely
 * */
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::socialForce(DecisionGraph* dg) {
  Tvector force;
  Person ego = dg->person_list_[id];
  // set according to Moussaid-Helbing 2009
  const double lambdaImportance = 2.0; // define relative importance of position vs velocity vector
  const double gamma = 0.35; // define speed interaction
  const double n = 2; // define speed interaction
  const double n_prime = 3; // define angular interaction  
  const double neighbor_dis = 3.0; // define max distance
  for (const Ped::Tagent* neighbor : neighbors) {
    Person other = dg->person_list_[neighbor->id];
    if (other.id == id) continue; //排除自己
    if (hypot(other.x-ego.x, other.y-ego.y)>neighbor_dis) continue; //忽略距离太远的人
    // ego和other不在同一个edge且反向
    if (!(ego.current_edge.first==other.current_edge.first && ego.current_edge.second==other.current_edge.second || ego.current_edge.first==other.current_edge.second&&ego.current_edge.second==other.current_edge.first)){
      if (other.direction*ego.direction<0){
        // compute difference between both agents' positions
        Tvector diff = neighbor->p - p;
        Tvector diffDirection = diff.normalized();
        // compute difference between both agents' velocity vectors
        // Note: the agent-other-order changed here
        Tvector velDiff = v - neighbor->v;
        // compute interaction direction t_ij
        Tvector interactionVector = diffDirection;
        Tvector interactionDirection = interactionVector / interactionVector.length();

        // compute angle theta (between interaction and position difference vector)
        Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

        // compute model parameter B = gamma * ||D||
        double B = gamma * interactionVector.length();
        double thetaRad = theta.toRadian();
        double forceVelocityAmount =
            -exp(-diff.length() / B -
                (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
        double forceAngleAmount =
            -exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

        Tvector forceVelocity = forceVelocityAmount * interactionDirection;
        Tvector forceAngle =
            forceAngleAmount * interactionDirection.leftNormalVector();

        force += forceVelocity + 5*forceAngle;
        // force += forceAngle;

      }
    }
    // 在同一个双车道的forward edge上
    else if (dg->person_list_[id].state == "onForwardEdge" && dg->graph_.getEdges()[ego.current_edge.first][ego.current_edge.second].getWidthAttribute()=="double"){
      // oncoming pedestrians：反向相反且还没meet
      if (other.direction * ego.direction<=0 && (other.x-ego.x)*ego.direction>0){
          //push laterally
        int& start_x = dg->graph_.getNodeList()[ego.current_edge.first].x;
        int& start_y = dg->graph_.getNodeList()[ego.current_edge.first].y;
        int& target_x = dg->graph_.getNodeList()[ego.current_edge.second].x;
        int& target_y = dg->graph_.getNodeList()[ego.current_edge.second].y;
        // F1: 力的方向和voronoiforce垂直
        force += Tvector(voronoiforce.y, -voronoiforce.x).normalized() * fabs(hypot(other.x-ego.x, other.y-ego.y)-1.5);
        // F2: 力的方向和人垂直
        // compute difference between both agents' positions
        Tvector diff = neighbor->p - p;
        Tvector diffDirection = diff.normalized();
        // compute difference between both agents' velocity vectors
        // Note: the agent-other-order changed here
        Tvector velDiff = v - neighbor->v;
        // compute interaction direction t_ij
        Tvector interactionVector = diffDirection;
        Tvector interactionDirection = interactionVector / interactionVector.length();

        // compute angle theta (between interaction and position difference vector)
        Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

        // compute model parameter B = gamma * ||D||
        double B = gamma * interactionVector.length();
        double thetaRad = theta.toRadian();
        double forceAngleAmount =
            -exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));
        Tvector forceAngle =
            forceAngleAmount * interactionDirection.leftNormalVector();
        force += 8*forceAngle;
        
      }
      // overtaking：方向相同、在同一edge、ego在后、ego速度快
      else if (other.direction*ego.direction>0 
      && ego.current_edge.first==other.current_edge.first && ego.current_edge.second==other.current_edge.second 
      && (other.x-ego.x)*ego.direction>0
      && hypot(ego.vx,ego.vy)>hypot(other.vx, other.vy)+0.1){
        int& start_x = dg->graph_.getNodeList()[ego.current_edge.first].x;
        int& start_y = dg->graph_.getNodeList()[ego.current_edge.first].y;
        int& target_x = dg->graph_.getNodeList()[ego.current_edge.second].x;
        int& target_y = dg->graph_.getNodeList()[ego.current_edge.second].y;
        force += Tvector(-voronoiforce.y, voronoiforce.x).normalized() * fabs(hypot(other.x-ego.x, other.y-ego.y)-1.5);
      }
      //overtaked：方向相同、在同一edge、ego在前、ego速度慢
      else if (other.direction*ego.direction>0 
      && ego.current_edge.first==other.current_edge.first && ego.current_edge.second==other.current_edge.second 
      && (other.x-ego.x)*ego.direction<0
      && hypot(ego.vx,ego.vy)+0.1<hypot(other.vx, other.vy)){
        int& start_x = dg->graph_.getNodeList()[ego.current_edge.first].x;
        int& start_y = dg->graph_.getNodeList()[ego.current_edge.first].y;
        int& target_x = dg->graph_.getNodeList()[ego.current_edge.second].x;
        int& target_y = dg->graph_.getNodeList()[ego.current_edge.second].y;
        force += Tvector(voronoiforce.y, -voronoiforce.x).normalized() * fabs(hypot(other.x-ego.x, other.y-ego.y)-1.5);
      }
    }
    // 在ending edge上，推动人走
    else if (dg->person_list_[id].state == "onEndingEdge"){
      Tvector diff = neighbor->p - p;
      Tvector diffDirection = diff.normalized();
      // compute difference between both agents' velocity vectors
      // Note: the agent-other-order changed here
      Tvector velDiff = v - neighbor->v;
      // compute interaction direction t_ij
      Tvector interactionVector = diffDirection;
      Tvector interactionDirection = interactionVector / interactionVector.length();

      // compute angle theta (between interaction and position difference vector)
      Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

      // compute model parameter B = gamma * ||D||
      double B = gamma * interactionVector.length();
      double thetaRad = theta.toRadian();
      double forceVelocityAmount =
          -exp(-diff.length() / B -
              (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
      double forceAngleAmount =
          -exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

      Tvector forceVelocity = forceVelocityAmount * interactionDirection;
      Tvector forceAngle =
          forceAngleAmount * interactionDirection.leftNormalVector();

      force += forceVelocity;
    }
    // overlap时的减速和加速
    if (hypot(other.x-ego.x,other.y-ego.y)<0.3*2+0.1){  //人的半径为0.3/0.1
      Tvector diff = neighbor->p - p;
      Tvector diffDirection = diff.normalized();
      // compute difference between both agents' velocity vectors
      // Note: the agent-other-order changed here
      Tvector velDiff = v - neighbor->v;
      // compute interaction direction t_ij
      Tvector interactionVector = diffDirection;
      Tvector interactionDirection = interactionVector / interactionVector.length();

      // compute angle theta (between interaction and position difference vector)
      Ped::Tangle theta = interactionDirection.angleTo(diffDirection);

      // compute model parameter B = gamma * ||D||
      double B = gamma * interactionVector.length();
      double thetaRad = theta.toRadian();
      double forceVelocityAmount =
          -exp(-diff.length() / B -
              (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
      double forceAngleAmount =
          -exp(-diff.length() / B - (n * B * thetaRad) * (n * B * thetaRad));

      Tvector forceVelocity = forceVelocityAmount * interactionDirection;
      Tvector forceAngle =
          forceAngleAmount * interactionDirection.leftNormalVector();

      force += 3*forceVelocity;      
    }
  }

  return force;
}

/// Calculates the force between this agent and the nearest obstacle in this
/// scene.
/// Iterates over all obstacles == O(N).
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::obstacleForce() const {
  // obstacle which is closest only
  Ped::Tvector minDiff;
  double minDistanceSquared = INFINITY;
  for (const Tobstacle* obstacle : scene->obstacles) {
    Ped::Tvector closestPoint = obstacle->closestPoint(p);
    Ped::Tvector diff = p - closestPoint;
    double distanceSquared = diff.lengthSquared();  // use squared distance to
    // avoid computing square
    // root
    if (distanceSquared < minDistanceSquared) {
      minDistanceSquared = distanceSquared;
      minDiff = diff;
    }
  }

  double distance = sqrt(minDistanceSquared) - agentRadius;
  double forceAmount = exp(-distance / forceSigmaObstacle);
  return forceAmount * minDiff.normalized();
}

/// myForce() is a method that returns an "empty" force (all components set to
/// 0).
/// This method can be overridden in order to define own forces.
/// It is called in move() in addition to the other default forces.
/// \return  Tvector: the calculated force
/// \param   e is a vector defining the direction in which the agent wants to
/// walk to.
Ped::Tvector Ped::Tagent::myForce(Ped::Tvector e) const {
  return Ped::Tvector();
}

void Ped::Tagent::computeForces(DecisionGraph* dg) {
  // update neighbors
  // NOTE - have a config value for the neighbor range
  const double neighborhoodRange = 100.0;
  neighbors = scene->getNeighbors(p.x, p.y, neighborhoodRange);
  // update forces
  // 以最大速度走向目标waypoint的力
  desiredforce = desiredForce();
  // 计算人与人之间的斥力
  if (forceFactorObstacle > 0) obstacleforce = obstacleForce();
  if (forceFactorVoronoi>0) voronoiforce = voronoiForce(dg);
  if (forceFactorSocial > 0) socialforce = socialForce(dg);
  myforce = myForce(desiredDirection);
}

/// Does the agent dynamics stuff. Calls the methods to calculate the individual
/// forces, adds them
/// to get the total force affecting the agent. This will then be translated
/// into a velocity difference,
/// which is applied to the agents velocity, and then to its position.
/// \param   stepSizeIn This tells the simulation how far the agent should
/// proceed

void Ped::Tagent::move(double stepSizeIn, DecisionGraph *dg) {
  // sum of all forces --> acceleration
  // a = forceFactorDesired * desiredforce + forceFactorSocial * socialforce +
  //     forceFactorObstacle * obstacleforce + forceFactorVoronoi*voronoiforce + myforce;
  // std::cout<<std::endl<<"--- move ---"<<std::endl;
  // std::cout<<forceFactorDesired<<", "<<forceFactorSocial<<", "<<forceFactorObstacle<<", "<<forceFactorVoronoi<<std::endl;
  // a = forceFactorVoronoi*voronoiforce;
  double max_acc = 1.0;
  Ped::Tvector temp_a = forceFactorSocial * socialforce + forceFactorVoronoi*voronoiforce;
  if (temp_a.length()>=max_acc) temp_a = max_acc * temp_a.normalized();
  // don't exceed maximal speed
  Ped::Tvector temp_v = v + stepSizeIn * temp_a;
  if (temp_v.length() > vmax) temp_v = temp_v.normalized() * vmax; 
  Ped::Tvector temp_p = p + stepSizeIn * temp_v;    // with larger step
  int mx_current, my_current;
  dg->graph_.world2map(mx_current,my_current,p.x,p.y);
  int mx, my;
  dg->graph_.world2map(mx,my,temp_p.x, temp_p.y);
  if (mx==mx_current && my==my_current && temp_v.length()!=0){
    if (temp_v.x>temp_v.y)
      ++mx;
    else if (temp_v.x==temp_v.y){
      ++mx;
      ++my;
    }
    else
      ++my;
  }

  if (!dg->passable_map_[mx][my]){
    // std::cout<<"("<<p.x<<","<<p.y<<") ("<<temp_p.x<<","<<temp_p.y<<")"<<std::endl;
    // std::cout<<"("<<mx_current<<","<<my_current<<") ("<<mx<<","<<my<<")"<<std::endl;

    //沿障碍物做分解
    float min_dis=999;
    float min_dx, min_dy;
    std::vector<std::pair<int,int>> iter = {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}};
    // std::vector<std::pair<int,int>> res;
    // for (auto it:iter)
    //   if (dg->distance_map_[mx+it.first][my+it.second] == dis_value)
    //     res.push_back(it);
    // std::cout<<"-**----------- "<<id<<"  "<<res.size()<<std::endl;
    // assert(res.size()>=2);
    // Ped::Tvector wall_vec(res.back().first-res.front().first, res.back().second-res.front().second);
    for (auto it:iter){
      if ((!dg->passable_map_[mx+it.first][my+it.second]) && hypot(it.first,it.second)<=min_dis && dg->passable_map_[mx-it.first][my-it.second]){
        min_dis = hypot(it.first,it.second);
        min_dx = it.first;
        min_dy = it.second;
      }
    }
    Ped::Tvector wall_vec(min_dx, min_dy);
    
    wall_vec.normalized().leftNormalVector();
    double cos_value = wall_vec.x*temp_v.x + wall_vec.y*temp_v.y;
    int dx,dy;
    if (fabs(wall_vec.x)>fabs(wall_vec.y)){
      dx=1;
      dy=0;
    }
    else if (fabs(wall_vec.x)<fabs(wall_vec.y)){
      dx=0;
      dy=1;
    }
    else{
      dx=1;
      dy=1;
    }
    dx *= dg->sign(wall_vec.x);
    dy *= dg->sign(wall_vec.y);
    if (!dg->passable_map_[mx+dx][my+dy]){
      wall_vec = - wall_vec;
      dx = -dx;
      dy = -dy;
    }
    // assert(dg->passable_map_[mx+dx][my+dy]);

    temp_v = temp_v + 1.5*wall_vec*fabs(cos_value);
    temp_v = temp_v.normalized()*vmax;
    // std::cout<<dg->distance_map_[mx][my]<<": "<<temp_v.x<<","<<temp_v.y<<std::endl;
  }

  a = (temp_v-v)/stepSizeIn;
  v = temp_v;
  p += stepSizeIn*v;
  theta = std::atan2(v.y, v.x);
  
  // notice scene of movement
  scene->moveAgent(this);
  // std::cout<<"--- move end ---"<<std::endl;
}


void Ped::Tagent::move(double stepSizeIn, pair<int,int> action){
  // calculate the new velocity
  float LIN_VEL_STEP_SIZE = 0.02;
  float ANG_VEL_STEP_SIZE = 0.02;
  if (action.first==-2 && action.second==-2){
    v=0*v;
    return;
  }
  theta += action.second*ANG_VEL_STEP_SIZE;

  if (v.x*cos(theta)+v.y*sin(theta) > 0){
    v = (v.length()+ action.first*LIN_VEL_STEP_SIZE)*Ped::Tvector(cos(theta),sin(theta),0);
  }
  else{
    v = (-v.length()+ action.first*LIN_VEL_STEP_SIZE)*Ped::Tvector(cos(theta),sin(theta),0);
  }
  
  // don't exceed maximal speed
  double speed = v.length();
  if (speed > vmax) v = v.normalized() * vmax;
  // internal position update = actual move
  p += stepSizeIn * v;
  // notice scene of movement
  scene->moveAgent(this);
}

void Ped::Tagent::move(Ped::Tvector state){
  v = Ped::Tvector(state.vx, state.vy, 0.0);
  p = Ped::Tvector(state.x, state.y, 0.0);
  theta = state.yaw;
  scene->moveAgent(this);
}