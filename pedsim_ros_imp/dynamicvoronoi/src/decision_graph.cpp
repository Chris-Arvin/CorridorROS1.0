#include <dynamicvoronoi/decision_graph.h>


Graph::Graph(){
    n_=0;
};

void Graph::addNode(Node p){
    node_list_.push_back(p);
    ++n_;
}

void Graph::addEdge(std::vector<std::pair<int,int>> edge, float road_width){
    int index1 = nodeToIndex(Node(edge.front().first,edge.front().second));
    int index2 = nodeToIndex(Node(edge.back().first,edge.back().second));
    // std::cout<<"-- "<<index1<<": "<<edge.front().first<<","<<edge.front().second<<std::endl;
    // std::cout<<"-- "<<index2<<": "<<edge.back().first<<","<<edge.back().second<<std::endl;
    // std::cout<<edge.size()<<std::endl;
    // node存在
    assert(index1!=-1 && index2!=-1);
    // node仅被访问一次
    assert(!edges_[index1][index2].isInitilized());
    assert(!edges_[index2][index1].isInitilized());

    auto edge_copy = edge;
    reverse(edge_copy.begin(), edge_copy.end());
    edges_[index1][index2] = Edge(edge, road_width);
    edges_[index2][index1] = Edge(edge_copy, road_width);

    // for (auto p:edge)
    //     edge_labeled_map_[p.first][p.second] = std::make_pair(index1,index2);
    // edge_labeled_map_[edge.front().first][edge.front().second] = std::make_pair(index1,index1);
    // edge_labeled_map_[edge.back().first][edge.back().second] = std::make_pair(index2,index2);
}

void Graph::setMap(std::vector<std::vector<bool>> voronoi_map, nav_msgs::OccupancyGrid global_map){
    voronoi_map_ = voronoi_map;
    global_map_ = global_map;
    edge_labeled_map_ = std::vector<std::vector<std::pair<int,int>>> (voronoi_map.size(), std::vector<std::pair<int,int>>(voronoi_map.front().size(), std::make_pair(-1,-1)));
}


Edge Graph::getEdge(Node from, Node to){
    int index1 = nodeToIndex(from);
    int index2 = nodeToIndex(to);
    return edges_[index1][index2];
}

Edge Graph::getEdge(int index1, int index2){
    return edges_[index1][index2];
}

int Graph::nodeToIndex(const Node& p){
    int count=-1;
    for (int i=0; i<n_; i++)
        if (node_list_[i]==p)
            count = i;
    return count;
}

void Graph::finishAddNode(){
    // initialize the graph
    edges_.clear();
    for (int i=0; i<n_; i++){
        // std::cout<<i<<","<<n_<<std::endl;
        std::vector<Edge> temp1;
        Edge temp2;
        for (int j=0; j<n_; j++)
            temp1.push_back(temp2);        
        edges_.push_back(temp1);
    }
}

void Graph::assignAttributeforNode(){
    // node的edge只能是1或者3
    for (int i=0; i<n_; i++){
        int num=0;
        for (int j=0; j<n_; j++){
            if (i==j)
                continue;
            if (edges_[i][j].isInitilized())
                ++num;
        }
        if (num==1)
            node_list_[i].attribute_ = "end";
        if (num==3)
            node_list_[i].attribute_ = "interval";        
    }
    if (n_>=2){
        node_list_[0].attribute_ = "interval";
        node_list_[n_-1].attribute_ = "interval";
    }
    // check
    // std::cout<<"****"<<std::endl;
    for (int i=0; i<n_; i++){
        // std::cout<<i<<": "<<node_list_[i].attribute_<<std::endl;
        assert(node_list_[i].attribute_ != "");
    }
}

void Graph::assignAttributeforEdge(){
    // std::cout<<"----"<<std::endl;
    // for (int i=0; i<n_; i++){
    //     for (int j=0; j<n_; j++){
    //         if (edges_[i][j].isInitilized())
    //             std::cout<<1<<" ";
    //         else
    //             std::cout<<0<<" ";
    //     }
    //     std::cout<<std::endl;
    // }


    for (int i=0; i<n_; i++){
        for (int j=i+1; j<n_; j++){
            if (!edges_[i][j].isInitilized())
                continue;
            // 两个边的属性不同
            if (node_list_[i].attribute_=="end" && node_list_[j].attribute_=="interval"){
                edges_[i][j].setDirAttribute("backtoway");
                edges_[j][i].setDirAttribute("giveway");
            }
            else if (node_list_[i].attribute_=="interval" && node_list_[j].attribute_=="end"){
                edges_[i][j].setDirAttribute("giveway");
                edges_[j][i].setDirAttribute("backtoway");
            }
            //两个边的属性相同
            else if (node_list_[i].attribute_=="interval" && node_list_[j].attribute_=="interval" && node_list_[i].x<=node_list_[j].x){
                edges_[i][j].setDirAttribute("forward");
                edges_[j][i].setDirAttribute("backward");
            }
            else if (node_list_[i].attribute_=="interval" && node_list_[j].attribute_=="interval" && node_list_[i].x>node_list_[j].x){
                edges_[i][j].setDirAttribute("backward");
                edges_[j][i].setDirAttribute("forward");
            }
            else if (node_list_[i].attribute_=="end" && node_list_[j].attribute_=="end" && node_list_[i].x<=node_list_[j].x){
                edges_[i][j].setDirAttribute("forward");
                edges_[j][i].setDirAttribute("backward");
            }
            else if (node_list_[i].attribute_=="end" && node_list_[j].attribute_=="end" && node_list_[i].x>node_list_[j].x){
                edges_[i][j].setDirAttribute("backward");
                edges_[j][i].setDirAttribute("forward");
            }
        }
    }
}

void Graph::map2world(int mx, int my, double& wx, double& wy){
    wx = global_map_.info.origin.position.x + mx*global_map_.info.resolution + 0.5*global_map_.info.resolution;
    wy = global_map_.info.origin.position.y + my*global_map_.info.resolution + 0.5*global_map_.info.resolution;    
}

void Graph::world2map(int& mx, int& my, double wx, double wy){
    mx = (wx-global_map_.info.origin.position.x-0.5*global_map_.info.resolution)/global_map_.info.resolution;
    my = (wy-global_map_.info.origin.position.y-0.5*global_map_.info.resolution)/global_map_.info.resolution;
    // std::cout<<"*** "<<wx<<","<<wy<<","<<global_map_.info.origin.position.x<<","<<global_map_.info.origin.position.y<<","<<global_map_.info.resolution<<";"<<mx<<","<<my<<std::endl;
}

bool Graph::checkInMap(int mx, int my){
    if (mx<0 || my<0 || mx>=voronoi_map_.size() || my >=voronoi_map_.front().size())
        return false;
    return true;
}

std::pair<int,int> Graph::findNearestVoronoiPoint(float wx, float wy){
    int mx,my;
    world2map(mx,my,wx,wy);
    int shift_x = 0;
    int shift_y = 0;
    std::vector<std::pair<int,int>> fx = {{1,0}, {0,-1}, {-1,0}, {0,1}};
    int turn=0;
    std::vector<int> condition = {1,-1,-1,1};
    if (checkInMap(mx,my) && voronoi_map_[mx][my])
        return {mx,my};
    while (true){
        shift_x += fx[turn].first;
        shift_y += fx[turn].second;
        if (checkInMap(mx+shift_x,my+shift_y) && voronoi_map_[mx+shift_x][my+shift_y])
            return {mx+shift_x,my+shift_y};
        if (shift_x==condition[0] || shift_x==condition[2] || shift_y==condition[1] || shift_y==condition[3]){
            if (turn==0 || turn==3)
                condition[turn] += 1;
            else
                condition[turn] -= 1;
            turn = (turn+1)%4;
        }
    }
    return {-1,-1};
}

void Graph::removeRedundantEndingNodeAndEdge(){
    for(int i=0; i<n_; i++){    // interval点：node_list_[i]， 被留下的end node: node_list_[max_index], 
        // 检测所有interval点
        if (node_list_[i].attribute_ == "interval"){
            std::vector<int> end_node_index_list;
            std::vector<int> end_edge_length_list;
            // 记录interval到end点形成的edge
            for(int j=0; j<n_; j++){
                if (j==i) continue;
                if(node_list_[j].attribute_ == "end" && edges_[i][j].isInitilized()){
                    end_node_index_list.push_back(j);
                    end_edge_length_list.push_back(edges_[i][j].getLength());
                }
            }
            if(end_node_index_list.size()>1){
                int max_length = 0;
                int max_index;
                // 找到与interval形成最长路径的node的index: max_index
                for(int k=0; k<end_node_index_list.size(); k++){
                    if(end_edge_length_list[k]>max_length){
                        max_index = end_node_index_list[k];
                        max_length = end_edge_length_list[k];
                    }
                }
                // 将非最长路径的end node设置为dump，删除对应的edge
                for (int j:end_node_index_list){
                    if(j!=max_index){
                        node_list_[j].attribute_ = "dump";
                        edges_[i][j] = Edge();
                        edges_[j][i] = Edge();
                    }
                }
                // todo此时，此interval仅连接一个end node形成一个end edge，如果interval的另一边只有一个forward/backward的前序edge，则通过interval将这两个edge合并，如果有多个forward/back的前序edge，则不做合并处理
                std::vector<int> adjacent_interval_node_index_list;
                for(int k=0; k<node_list_.size(); k++)
                    if (node_list_[k].attribute_=="interval" && edges_[k][i].isInitilized())
                        adjacent_interval_node_index_list.push_back(k);
                if (adjacent_interval_node_index_list.size()==1){
                    // std::cout<<"++++ "<<i<<", "<<adjacent_interval_node_index_list.front()<<", "<<max_index<<std::endl;
                    // 赋值新edge
                    auto path1 = edges_[adjacent_interval_node_index_list.front()][i].getPath();
                    auto path2 = edges_[i][max_index].getPath();
                    // std::cout<<path1.size()<<". "<<path2.size()<<std::endl;
                    assert((path1.back().first==path2.front().first)&&(path1.back().second==path2.front().second));
                    for (int l=1; l<path2.size(); l++)
                        path1.push_back(path2[l]);
                    auto width1 = edges_[adjacent_interval_node_index_list.front()][i].getRoadWidth();
                    auto width2 = edges_[i][max_index].getRoadWidth();
                    edges_[adjacent_interval_node_index_list.front()][max_index] = Edge(path1,std::min(width1, width2));
                    edges_[adjacent_interval_node_index_list.front()][max_index].setDirAttribute("giveway");
                    auto path1_copy = path1;
                    reverse(path1_copy.begin(), path1_copy.end());
                    edges_[max_index][adjacent_interval_node_index_list.front()] = Edge(path1_copy,std::min(width1, width2));
                    edges_[max_index][adjacent_interval_node_index_list.front()].setDirAttribute("backtoway");
                    // std::cout<<edges_[max_index][adjacent_interval_node_index_list.front()].getPath().size()<<", "<<edges_[adjacent_interval_node_index_list.front()][max_index].getPath().size()<<std::endl;

                    // 删除旧的edge
                    edges_[adjacent_interval_node_index_list.front()][i] = Edge();
                    edges_[i][adjacent_interval_node_index_list.front()] = Edge();
                    edges_[i][max_index] = Edge();
                    edges_[max_index][i] = Edge();
                    node_list_[i].attribute_ = "dump";
                }

            }
        }
    }

    for (int i=0; i<n_; i++){
        for (int j=0; j<n_; j++){
            if (edges_[i][j].isInitilized()){
                auto edge = edges_[i][j].getPath();
                for (auto p:edge)
                    edge_labeled_map_[p.first][p.second] = std::make_pair(i,j);
                edge_labeled_map_[edge.front().first][edge.front().second] = std::make_pair(i,i);
                edge_labeled_map_[edge.back().first][edge.back().second] = std::make_pair(j,j);
            }
        }
    }
}


void Graph::clearEdges(){
    for (int i=0; i<n_; i++){
        for (int j=0; j<n_; j++){
            if (edges_[i][j].isInitilized()){
                edges_[i][j].clearCost();
                edges_[i][j].clearPerson();
            }
        }
    }
}

void Graph::addCostForEdge(int from, int to, float cost){
    edges_[from][to].addCost(cost);
}


void Graph::addPersonForEdge(std::pair<int,int>edge, std::pair<int,int>pos){
    edges_[edge.first][edge.second].addPerson(pos);
}


bool DecisionGraph::checkInMap(int x, int y){
    if (x<=0 || x>=nx_)
        return false;
    if (y<=0 || y>=ny_)
        return false;
    return true;
}

void DecisionGraph::constructDecisionGraph(std::vector<std::vector<bool>> voronoi_map, std::vector<std::vector<float>> distance_map, std::vector<std::vector<bool>> passable_map, nav_msgs::OccupancyGrid global_map){
    voronoi_map_ = voronoi_map;
    distance_map_ = distance_map;
    passable_map_ = passable_map;
    global_map_ = global_map;
    nx_ = voronoi_map.size();
    ny_ = voronoi_map.front().size();
    graph_.setMap(voronoi_map, global_map);
    std::vector<std::pair<int,int>> mask = {{0,1}, {0,-1}, {1,0}, {-1,0}};
    // extract nodes, and label them as true in node_map
    std::vector<std::vector<bool>> node_map(nx_, std::vector<bool>(ny_, false));
    std::vector<std::vector<bool>> visited_map(nx_, std::vector<bool>(ny_, false));
    std::vector<std::pair<int,int>> visited;
    for (int x=0; x<nx_; ++x){
        for (int y=0;y<ny_; ++y){
            // 当前点本身必须是voronoi点
            if (!voronoi_map[x][y])
                continue;
            // 邻居的voronoi点的个数应该为3（决策点）或1（端点）
            int neiborhghood = 0;
            for (auto p:mask){
                if (checkInMap(x+p.first,y+p.second) && voronoi_map_[x+p.first][y+p.second])
                    ++neiborhghood;
            }
            if (neiborhghood==3 || neiborhghood==1){
                node_map[x][y] = true;
                graph_.addNode(Node(x,y));
                if (visited.empty()){
                    visited.push_back({x,y});
                }
            }
        }
    }
    graph_.finishAddNode();
    // create graph: add vertices and edges to graph
    // std::cout<<"---- "<<std::endl;
    // for (auto nod:graph_.getNodeList())
    //     std::cout<<nod.x<<","<<nod.y<<"; ";
    // std::cout<<"---- "<<std::endl;
    depthFirst(visited, node_map, visited_map, mask);
    assignAttribute();
    // auto temp = graph_.getEdges();
    // for (int i=0; i<temp.size();i++){
    //     for(int j=0; j<temp.size(); j++){
    //         if(temp[i][j].isInitilized())
    //             std::cout<<temp[i][j].getRoadWidth()<<",  ";
    //     }
    //     std::cout<<std::endl;
    // }
    removeRedundantEndingNodeAndEdge();
    finishConstruct_ = true;
    std::cout<<"done"<<std::endl;
}

void DecisionGraph::removeRedundantEndingNodeAndEdge(){
    graph_.removeRedundantEndingNodeAndEdge();
}

void DecisionGraph::assignAttribute(){
    graph_.assignAttributeforNode();
    graph_.assignAttributeforEdge();
}

void DecisionGraph::updatePeople(int id, double x, double y, double vx, double vy, double direction){
    // std::cout<<"-----"<<std::endl;
    // std::cout<<id<<","<<x<<","<<y<<","<<direction<<std::endl;
    std::pair<int,int> pos = graph_.findNearestVoronoiPoint(x,y);
    std::pair<int,int> edge = graph_.findEdge(pos.first, pos.second);
    if (direction* (edge.second-edge.first)<0)
        edge = {edge.second,edge.first};
    // std::cout<<pos.first<<", "<<pos.second<<"; "<<edge.first<<", "<<edge.second<<"; "<<std::endl;
    // std::cout<<graph_.getNodeList().size()<<std::endl;
    std::string state;
    assert((edge.first!=-1)&&(edge.second!=-1));
    // std::cout<<"*** updatePeople"<<std::endl;
    if (edge.first==edge.second){
        if (graph_.getNodeList()[edge.first].attribute_=="interval"){
            state = "onForwardEdge";    //如果这样的话，在后续使用中edge的path为空，无所谓前序节点是什么了
            // std::cout<<1<<std::endl;
        }
        else{
            state = "onEndingEdge";
            const std::vector<Node>& node_list = graph_.getNodeList();
            std::vector<std::vector<Edge>> edges = graph_.getEdges();
            for (int i=0; i<node_list.size(); i++){
                Node p = node_list[i];
                if (p.attribute_=="interval" && edges[i][edge.first].isInitilized()){
                    edge = {edge.first, i}; //{endingNode, intervalNode}
                }            
            }
            // std::cout<<2<<" "<<edge.first<<","<<edge.second<<std::endl;
        }
    }
    else if (graph_.getEdges()[edge.first][edge.second].getDirAttribute()=="forward" || graph_.getEdges()[edge.first][edge.second].getDirAttribute()=="backward"){
        state = "onForwardEdge";
        // std::cout<<3<<std::endl;
    }
    else{
        state = "onEndingEdge";
        // std::cout<<4<<" "<<graph_.getEdges()[edge.first][edge.second].getDirAttribute()<<std::endl;
    }
    person_list_[id] = Person(id, x,y, vx, vy, (double)sign(direction),state, pos, edge);
    // if (id==6)  std::cout<<id<<": "<<edge.first<<","<<edge.second<<std::endl;
    // 以目标direction(destination)为第一准则
    if (direction*(edge.second-edge.first)>=0)
        graph_.addPersonForEdge(edge, pos);
    else
        graph_.addPersonForEdge({edge.second,edge.first}, pos);
    // std::cout<<"--- updatePeople: "<<id<<", "<<person_list_[id].direction<<std::endl;
}

std::pair<double, double> DecisionGraph::getDecision(int id){
    Person& person = person_list_[id];
    bool DEBUG=false;
    DEBUG = true;
    // if(id==2) DEBUG=true;
    // if (person.x>-5 && person.x<5) DEBUG=true;
    if(DEBUG) std::cout<<"*** getDecision: "<<id<<": "<<person.current_edge.first<<","<<person.current_edge.second<<"  "<<person.x<<","<<person.y<<std::endl;
    const std::vector<std::pair<int, int>>& path = graph_.getEdges()[person.current_edge.first][person.current_edge.second].getPath();
    const std::vector<Node>& node_list = graph_.getNodeList();
    std::vector<std::vector<Edge>> edges = graph_.getEdges();
    std::pair<int, int> next_pos;
    int LookForwardStep = 8;
    bool isMakeDecision = false;
    if (person.state == "onForwardEdge"){
        if(DEBUG) std::cout<<"onForwardEdge"<<std::endl;
        int index = 0;
        for (auto it=path.begin(); it!=path.end(); it++){
            if (path[index].first==person.nearest_voronoi_point.first && path[index].second==person.nearest_voronoi_point.second){
                break;
            }
            ++index;
        }
        // keep going，往前走或往后走
        if (index+LookForwardStep<path.size()){
            if(DEBUG) std::cout<<"  keep going: ";
            // 可前行
            if(edges[person.current_edge.first][person.current_edge.second].getWidthAttribute()=="double" || edges[person.current_edge.second][person.current_edge.first].getPersonNum()==0){
                next_pos = path[index+LookForwardStep];
                if(DEBUG) std::cout<<"forward";
            }
            // 考虑靠右行驶规则，违规的人需后退
            else{
                // 缓步前进，快速后退
                if (ceil(person.direction*edges[person.current_edge.second][person.current_edge.first].getMeanY()*0.1/1.5*LookForwardStep)>0)
                    LookForwardStep = (int)ceil(person.direction*edges[person.current_edge.second][person.current_edge.first].getMeanY()*0.1/1.5*LookForwardStep);   //分辨率0.1，路半宽1.5，向前看LookForwardStep长度
                else 
                    LookForwardStep = -LookForwardStep;
                next_pos = path[std::max(index+LookForwardStep,0)];
                if(DEBUG) std::cout<<"partionally forward or backward";
            }
            if(DEBUG) std::cout<<std::endl;
        }
        // make decision，准备进入下一个edge
        else{
            isMakeDecision = true;
            if(DEBUG) std::cout<<"  make decision"<<std::endl;
            float cost = 999.9;
            int decision_index = -1;
            LookForwardStep += 3;
            // 遍历接下来所有的edge
            for (int i=0; i<node_list.size(); i++){
                if(edges[person.current_edge.second][i].isInitilized()){
                    auto next_dir_attr = edges[person.current_edge.second][i].getDirAttribute();
                    if (next_dir_attr=="forward" || next_dir_attr=="backward"){
                        // std::cout<<person.direction<<"--"<<edges[person.current_edge.second][i].getMeanY()<<std::endl;
                        int cost_dir = sign(person.direction * (node_list[i].x-node_list[person.current_edge.second].x));
                        if (edges[person.current_edge.second][i].getWidthAttribute()=="double"){
                            // cost（越小越好）: edge被同向占据的情况，edge被逆向占据的情况，靠右行走特性，走向目标方向，todo group特性
                            // 权重：0.1，0.01，0.01, 0.1
                            if (DEBUG) std::cout<<i<<" double: "<<0.1*std::max(-1,(-edges[person.current_edge.second][i].getPersonNum()))<<", "<<0.01*edges[i][person.current_edge.second].getPersonNum()<<", "<<0.01*person.direction*edges[person.current_edge.second][i].getMeanY()/50.0<<", "<<0.1*(-cost_dir)<<std::endl;
                            float temp = 0.1*std::max(-1,(-edges[person.current_edge.second][i].getPersonNum())) + 0.01*edges[i][person.current_edge.second].getPersonNum() + 0.01*person.direction*edges[person.current_edge.second][i].getMeanY()/50.0 + 0.1*(-cost_dir);
                            if (temp<cost){
                                cost = temp;
                                decision_index = i;
                            }
                        }
                        else{   // single
                            // 权重：0，10，0.01, 0.1
                            if (DEBUG) std::cout<<i<<" single: "<<0.0*(-edges[person.current_edge.second][i].getPersonNum())<<", "<<10.0*edges[i][person.current_edge.second].getPersonNum()<<", "<<0.01*person.direction*edges[person.current_edge.second][i].getMeanY()/50.0<<", "<<0.1*(-cost_dir)<<std::endl;
                            float temp = 0.0*(-edges[person.current_edge.second][i].getPersonNum()) + 10.0*edges[i][person.current_edge.second].getPersonNum() + 0.01*person.direction*edges[person.current_edge.second][i].getMeanY()/50.0 + 0.1*(-cost_dir);
                            if (temp<cost){
                                cost = temp;
                                decision_index = i;
                            }
                        }
                    }
                    else if (next_dir_attr=="giveway"){
                        // todo 这个权重好像不是这么设置的
                        float temp = 3-std::min(3.0, edges[person.current_edge.second][i].getLength()*0.1)/(edges[person.current_edge.second][i].getPersonNum()+1);
                        if (temp<cost){
                            cost = temp;
                            decision_index = i;
                        }
                        if (DEBUG) std::cout<<i<<" giveway: "<<edges[person.current_edge.second][i].getLength()<<", "<<edges[person.current_edge.second][i].getPersonNum()<<", "<<temp<<std::endl;
                    }
                    else{
                        // 理论上应该不会到这一步
                        assert(("error: from forward to giveway" && false));
                    }

                }
            }
            // std::cout<<"size: "<<edges[person.current_edge.second][decision_index].getPath().size()<<std::endl;
            // 倒退
            if (decision_index==person.current_edge.first)
                next_pos = edges[person.current_edge.second][decision_index].getPath()[std::min(index+LookForwardStep, (int)edges[person.current_edge.second][decision_index].getPath().size()-1)];
            // 前进，选择其他edge
            else{
                int mx,my;
                graph_.world2map(mx,my,person.x,person.y);

                next_pos = edges[person.current_edge.second][decision_index].getPath()[std::min(LookForwardStep - (int)(path.size()-index-1), (int)edges[person.current_edge.second][decision_index].getPath().size()-1)];
                std::pair<int, int> final_pos = edges[person.current_edge.second][decision_index].getPath().back();
                float len = hypot(next_pos.second-final_pos.second, next_pos.first-final_pos.first);
                if (len ==0 ){
                    final_pos = {mx,my};
                    len = hypot(next_pos.second-final_pos.second, next_pos.first-final_pos.first);
                }
                std::pair<float, float> vertical_vec1 = {(float)(next_pos.second-final_pos.second)/len, -(float)(next_pos.first-final_pos.first)/len};
                std::pair<float, float> vertical_vec2 = {-(float)(next_pos.second-final_pos.second)/len, (float)(next_pos.first-final_pos.first)/len};

                if (hypot((next_pos.first-mx)+vertical_vec1.first,(next_pos.second-my)+vertical_vec1.second) < hypot((next_pos.first-mx)+vertical_vec2.first,(next_pos.second-my)+vertical_vec2.second))
                    vertical_vec2 = vertical_vec1;
                float vertical_len_m = std::min(edges[person.current_edge.second][decision_index].getRoadWidth()/2-3-2, fabs(vertical_vec2.first*(float)(next_pos.first-mx) + vertical_vec2.second*(float)(next_pos.second-my)));   //人的宽度为0.3/0.1
                
                vertical_vec2 = {fabs(vertical_len_m)*vertical_vec2.first, fabs(vertical_len_m)*vertical_vec2.second};
                next_pos = {next_pos.first + vertical_vec2.first, next_pos.second+vertical_vec2.second};
                // std::cout<<"-----------"<<person.id<<": ("<<next_pos.first<<" + "<<vertical_vec2.first<<") ("<<next_pos.second<<" + "<<vertical_vec2.second<<")"<<std::endl;
            }
            assert(decision_index!=-1);
            if(DEBUG) std::cout<<"decision_index: "<<decision_index<<"; "<<person.current_edge.first<<","<<person.current_edge.second<<std::endl;
        }

    }
    else if (person.state == "onEndingEdge"){
        if(DEBUG) std::cout<<"onEndingEdge"<<std::endl;
        std::vector<int> can_go_index_list;
        std::vector<float> cost_list;   // 越大越好
        // 若回退到上一个interval节点，该节点的index
        int interval_index, ending_index;
        for (int i=0; i<node_list.size(); i++){
            Node p = node_list[i];
            if (p.attribute_=="interval"){
                if(i==person.current_edge.first){
                    interval_index = i;
                    ending_index = person.current_edge.second;
                    break;
                }
                else if (i==person.current_edge.second){
                    interval_index = i;
                    ending_index = person.current_edge.first;
                }
            }            
        }
        // 回退后 如果选择向前走，则需前向车道为double 或 single且无对向来人
        for (int i=0; i<node_list.size(); i++){
            if(edges[interval_index][i].isInitilized()){
                auto next_dir_attr = edges[interval_index][i].getDirAttribute();
                if (next_dir_attr=="forward" || next_dir_attr=="backward"){
                    int cost_dir = sign(person.direction * (node_list[i].x-node_list[interval_index].x));
                    // 双向车道 或者 对面没人，则可退
                    if (edges[interval_index][i].getDirection()*person.direction>0 && (edges[interval_index][i].getWidthAttribute()=="double" || edges[i][interval_index].getPersonNum()==0)){
                        can_go_index_list.push_back(i);
                        float temp_cost;
                        if (edges[interval_index][i].getWidthAttribute()=="double"){
                            // cost（越大越好）: edge被同向占据的情况，edge被逆向占据的情况，靠右行走特性，todo group特性
                            // 权重：0.1，0.01，0.01, 0.1
                            temp_cost = 0.1*std::max(-1,(-edges[interval_index][i].getPersonNum())) + 0.01*edges[i][interval_index].getPersonNum() + 0.01*person.direction*edges[interval_index][i].getMeanY()/50.0 + 0.1*(-cost_dir);
                        }
                        else if (edges[interval_index][i].getWidthAttribute()=="single")
                            // 权重：0，10，0.01, 0.1
                            temp_cost = 0.0*(-edges[interval_index][i].getPersonNum()) + 10.0*edges[i][interval_index].getPersonNum() + 0.01*person.direction*edges[interval_index][i].getMeanY()/50.0 + 0.1*(-cost_dir);
                        else
                            assert((edges[interval_index][i].getWidthAttribute()=="single" || edges[interval_index][i].getWidthAttribute()=="double"));
                        cost_list.push_back(temp_cost);
                    }
                }
            }
        }
        // 不可回退，则在最近的voronoi点stay （同时被其他人push）
        if (can_go_index_list.empty()){
            if(DEBUG) std::cout<<"  cannot go back"<<std::endl;
            int index = 0;
            auto path = graph_.getEdges()[ending_index][interval_index].getPath();
            for (auto it=path.begin(); it!=path.end(); it++){
                if (path[index].first==person.nearest_voronoi_point.first && path[index].second==person.nearest_voronoi_point.second){
                    break;
                }
                ++index;
            }
            std::cout<<"-*-*-*-*-*-* "<<index<<", "<<path.size()<<std::endl;
            next_pos = path[index];
        }
        // 可回退，则更退 / 重新决策
        else{
            if(DEBUG) std::cout<<"  can go back"<<std::endl;
            int index = 0;
            int decision_index = -1;
            auto path = graph_.getEdges()[ending_index][interval_index].getPath();
            for (auto it=path.begin(); it!=path.end(); it++){
                if (path[index].first==person.nearest_voronoi_point.first && path[index].second==person.nearest_voronoi_point.second){
                    break;
                }
                ++index;
            }
            // keep going
            if (index+LookForwardStep<path.size()){
                if(DEBUG) std::cout<<"    keep going"<<std::endl;
                next_pos = path[index+LookForwardStep];           
            }
            // make decision
            else{
                isMakeDecision = true;
                LookForwardStep = std::max(3,LookForwardStep-3);
                if(DEBUG) std::cout<<"    make decision"<<std::endl;
                float temp_cost = 999.9;
                for (int j=0; j<can_go_index_list.size(); j++){
                    if (cost_list[j]<temp_cost){
                        decision_index = can_go_index_list[j];
                        temp_cost = cost_list[j];
                    }
                    
                }
                assert(decision_index!=-1);
                if (LookForwardStep-(path.size()-index-1) < edges[interval_index][decision_index].getPath().size())
                    next_pos = edges[interval_index][decision_index].getPath()[LookForwardStep - (path.size()-index-1)];
                else
                    next_pos = edges[interval_index][decision_index].getPath().back();            
            }
        }
    }
    double next_wx, next_wy;
    double nearest_wx, nearest_wy;
    graph_.map2world(next_pos.first, next_pos.second, next_wx, next_wy);
    graph_.map2world(person.nearest_voronoi_point.first, person.nearest_voronoi_point.second, nearest_wx, nearest_wy);
    if(DEBUG) std::cout<<"getDecision ***"<<std::endl;
    if (isMakeDecision){
        double len = hypot(next_wx-person.x, next_wy-person.y);
        return {(next_wx-person.x)/len*0.5, (next_wy-person.y)/len*0.5};
    }
    else{
        double len = hypot(next_wx-nearest_wx + 0.8*(nearest_wx-person.x), next_wy-nearest_wy + 0.8*(nearest_wy-person.y));      
        return {(next_wx-nearest_wx + 0.8*(nearest_wx-person.x))/len, (next_wy-nearest_wy + 0.8*(nearest_wy-person.y))/len};
    }
}


void DecisionGraph::depthFirst(std::vector<std::pair<int,int>>& visited, const std::vector<std::vector<bool>>& node_map, std::vector<std::vector<bool>>& visited_map, const std::vector<std::pair<int,int>>& delta)
{
    auto back = visited.back();
    // 两个节点已经连接，生成edge
    if (node_map[back.first][back.second] && visited.size()>=2){
        std::vector<std::pair<int,int>> temp_edge = {back};
        auto it1 = visited.end();
        std::advance(it1,-2);
        auto it2 = visited.begin();
        while (true){
            temp_edge.push_back(*it1);
            if (node_map[it1->first][it1->second])
                break;
            it1--;                              
            if (it1==it2){
                temp_edge.push_back(*it1);
                break;
            }
        }
        if (temp_edge.front().first==temp_edge.back().first && temp_edge.front().second==temp_edge.back().second)
            return;
        else{
            assert(temp_edge.size()>=2);
            float dist = distance_map_[temp_edge.front().first][temp_edge.front().second];
            for (auto p:temp_edge){
                dist = std::min(dist,distance_map_[p.first][p.second]);
            }
            graph_.addEdge(temp_edge, dist*2);
        }
    }
    
    for (auto it=delta.begin(); it!=delta.end(); ++it){
        int current_x = back.first+it->first;
        int current_y = back.second+it->second;
            
        // 保证点在地图中且被voronoi标记
        if (!checkInMap(current_x, current_y) || !voronoi_map_[current_x][current_y])
            continue;
        // 点是否已经被visited了 && 点是否是node点
        if (visited_map[current_x][current_y] && !node_map[current_x][current_y])
            continue;
        // 防止两个相邻的node点被重复处理
        if (visited_map[current_x][current_y] && node_map[current_x][current_y] && node_map[back.first][back.second])
            continue;

        visited.push_back(std::make_pair(current_x, current_y));
        // if(!node_map[current_x][current_y])
        visited_map[current_x][current_y] = true;
        depthFirst(visited, node_map, visited_map, delta);
        visited.pop_back();
    }
}

int DecisionGraph::sign(float x){
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

