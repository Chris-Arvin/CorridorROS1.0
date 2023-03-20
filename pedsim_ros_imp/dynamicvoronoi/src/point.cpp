#include <dynamicvoronoi/point.h>

std::pair<int,int> Node::getPos() {return std::make_pair(x,y);};
void Node::setPos(int _x, int _y) {x=_x; y=_y;};
bool Node::operator==(const Node& temp)const{ return temp.x==this->x && temp.y==this->y; };

Edge::Edge(std::vector<std::pair<int,int>> pos_list, float road_width){
  road_width_ = road_width;
  pos_list_ = pos_list;
  length_ = pos_list.size();
  mean_y_ = 0;
  for (auto p:pos_list_)
    mean_y_ += p.second*1.0;
  mean_y_ /= pos_list_.size();
  if (road_width_>0.3/0.1*4) // 设人的半径为0.3
    width_attr_ = "double";
  else
    width_attr_ = "single";
  cost_ = 0.0;
  direction_ = pos_list_.back().first-pos_list_.front().first;
};
bool Edge::isInitilized() {return pos_list_.empty()?false:true;};
void Edge::setDirAttribute(std::string attr){dir_attr_=attr;};
void Edge::addPerson(std::pair<int,int>pos) {person_list_.push_back(pos);};
