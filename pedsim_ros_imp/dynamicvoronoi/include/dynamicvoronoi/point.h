#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
#include <ros/ros.h>
#include <algorithm>

class IntPoint {
public:
  IntPoint() : x(0), y(0) {};
  IntPoint(int _x, int _y) : x(_x), y(_y) {};
  int x,y;
};

class Node{
public:
  Node(){};
  Node(int _x, int _y):x(_x),y(_y){};
  std::pair<int,int> getPos();
  void setPos(int _x, int _y);
  bool operator==(const Node& temp)const;
  int x;
  int y;
  std::string attribute_ = ""; // interval, end, dump
};

class Edge{
public:
  Edge(){};
  Edge(std::vector<std::pair<int,int>> pos_list, float road_width);
  bool isInitilized();
  void setDirAttribute(std::string attr);
  void addCost(float cost) {cost_ += cost;};
  void addPerson(std::pair<int,int>pos);
  void clearCost() {cost_=0;};
  void clearPerson() {person_list_.clear();};
  float getMeanY() {return mean_y_;};
  float getCost() {return cost_;};
  int getPersonNum() {return person_list_.size();};
  int getLength() {return length_;};
  float getRoadWidth() {return road_width_;};
  std::string getDirAttribute() {return dir_attr_;};
  std::string getWidthAttribute() {return width_attr_;};
  std::vector<std::pair<int,int>> getPath() {return pos_list_;};
  int getDirection() {return direction_;};
private:
  std::vector<std::pair<int,int>> pos_list_;
  int length_;
  float road_width_;
  std::string dir_attr_; // [forward, backward, giveway, backtoway]
  std::string width_attr_; // [single, double]
  float cost_;
  std::vector<std::pair<int,int>> person_list_;
  float mean_y_;
  int direction_;
};

class Person{
public:
  Person(){};
  Person(int _id, double _x, double _y, double _vx, double _vy, double _direction, std::string _state, std::pair<int,int> _nearest_voronoi_point, std::pair<int,int> _current_edge):id(_id),x(_x),y(_y),vx(_vx),vy(_vy),direction(_direction), state(_state), nearest_voronoi_point(_nearest_voronoi_point), current_edge(_current_edge) {};
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double direction;
  int index;
  std::pair<int,int> nearest_voronoi_point;
  std::string state; // [onForwardEdge, onEndingEdge]
  std::pair<int,int> current_edge; // directional edge

};


#endif
