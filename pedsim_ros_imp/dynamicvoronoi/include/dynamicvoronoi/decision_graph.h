#ifndef _DECISIONGRAPH_H
#define _DECISIONGRAPH_H
/*********************************************************************
 *
 * Author: Zhang Qianyi
 * E-mail: zhangqianyi@mail.nankai.edu.cn
 * Last update: 2022.12.9
 *
 *********************************************************************/
#include <dynamicvoronoi/point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>

class Graph{
    public:
        Graph();
        void addNode(Node p);
        void addEdge(std::vector<std::pair<int,int>> e, float road_width);
        void setMap(std::vector<std::vector<bool>> voronoi_map, nav_msgs::OccupancyGrid global_map);
        void finishAddNode();
        int nodeToIndex(const Node& p);
        void assignAttributeforNode();
        void assignAttributeforEdge();
        void clearEdges();
        void addCostForEdge(int from, int to, float cost);        
        void addPersonForEdge(std::pair<int,int>edge, std::pair<int,int>pos);
        void removeRedundantEndingNodeAndEdge();   
        std::pair<int,int> findNearestVoronoiPoint(float wx, float wy);
        std::pair<int,int> findEdge(int mx, int my) {return edge_labeled_map_[mx][my];};

        Edge getEdge(Node p1, Node p2);
        Edge getEdge(int index1, int index2);
        std::vector<Node> getNodeList(){return node_list_;};
        std::vector<std::vector<Edge>> getEdges() {return edges_;};
        void map2world(int mx, int my, double& wx, double& wy);
        void world2map(int& mx, int& my, double wx, double wy);
        bool checkInMap(int mx, int my);

    private:
        std::vector<std::vector<Edge>> edges_;
        int n_;
        std::vector<Node> node_list_;
        std::vector<std::vector<bool>> voronoi_map_;
        std::vector<std::vector<std::pair<int,int>>> edge_labeled_map_;
        nav_msgs::OccupancyGrid global_map_;
};


class DecisionGraph{
    public:
        DecisionGraph(){finishConstruct_=false;};
        void constructDecisionGraph(std::vector<std::vector<bool>> voronoi_map, std::vector<std::vector<float>> distance_map, std::vector<std::vector<bool>> passable_map, nav_msgs::OccupancyGrid global_map);
        void depthFirst(std::vector<std::pair<int,int>>& visited, const std::vector<std::vector<bool>>& node_map, std::vector<std::vector<bool>>& visited_map, const std::vector<std::pair<int,int>>& delta);
        bool checkInMap(int x, int y);
        void assignAttribute();
        void removeRedundantEndingNodeAndEdge();
        void updatePeople(int id, double x, double y, double vx, double vy, double direction);
        std::pair<double, double> getDecision(int id);
        int sign(float x);

    public:
        std::vector<std::vector<bool>> voronoi_map_;
        std::vector<std::vector<float>> distance_map_;
        std::vector<std::vector<bool>> passable_map_;
        nav_msgs::OccupancyGrid global_map_;
        int nx_,ny_;
        bool finishConstruct_;
        Graph graph_;
        std::map<int, Person> person_list_;

};

#endif
