//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <cmath>
#include <map>
#include "distributor.h"


Distributor::Distributor() {
    std::shared_ptr<Context> sharedPtr = std::make_shared<Context>();
    this->context = sharedPtr;
    this->nodeTotalNum_ = this->context->GetNodeTotalNum();
    this->factoryIDShift_ = this->context->GetRobotTotalNum();

    this->historyGraph_ = DeepCopy2DVector(this->context->GetInitialHistoryGraph());
    this->globalGraph_ = DeepCopy2DVector(this->context->GetInitialHistoryGraph());

    std::cout << "OK" << std::flush; // 准备就绪，开始答题
}

bool Distributor::run() {
    while (this->context->UpdateAllStatus()){
        std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;

        // 根据广播更新地图
        this->UpdateFromBroadcast();






        if (this->context->GetFrameId() == 2){
            this->context->PrintHistoryMap(this->globalGraph_, "Distributor::globalGraph_");
            this->context->PrintHistoryMap(this->historyGraph_, "Distributor::historyGraph_");
        }

//        std::cerr << "FactoryType: " << this->context->GetFactory(9)->GetFactoryType() << std::endl;
//        auto a = FromIdTypeFindEdgeIndex(9, MATERIAL_1);


        //开始与控制台交互
        std::cout << this->context->GetFrameId() << std::endl;

//        this->context->GetRobot(0)->HighSpeedMove(40.0f, 40.0f, this->context->GetDt());
//        this->context->GetRobot(3)->HighSpeedMove(10.0f, 40.0f, this->context->GetDt());


//        if (!(this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(1), 8.5))){
//            std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
//        }

//        bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(3), 15);
//        if (aboutToCrash){
//            this->context->GetRobot(0)->Rotate(M_PI);
//            this->context->GetRobot(3)->Rotate(M_PI);
//        }else {
//            this->context->GetRobot(0)->HighSpeedMove(40.0f, 40.0f, this->context->GetDt());
//            this->context->GetRobot(3)->HighSpeedMove(10.0f, 40.0f, this->context->GetDt());

        // 两两检查是否相撞，共6次
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(i), this->context->GetRobot(j), 18);
                if (aboutToCrash){
                    this->context->GetRobot(i)->Rotate(M_PI);
                    this->context->GetRobot(j)->Rotate(M_PI);
//                    this->context->GetRobot(i)->Forward(0);
                }
            }
        }

        //与控制台交互结束
        std::cout << "OK" << std::flush;

        this->context->SetPreviousFrameId(this->context->GetFrameId());
    }
    return false;
}


bool Distributor::GraphOptimization() {


    return false;
}


void Distributor::DistributeTask(std::pair<double, std::vector<int>>& route) {
    int robotID = route.second[0];  // 机器人ID
    int thisBuyNode = route.second[1];  // 要去购买东西的节点ID
    int thisSellNode = route.second[2]; // 要去卖东西的节点ID
    FactoryType sellNodeType = this->context->GetFactory(thisSellNode)->GetFactoryType();
    FactoryType buyNodeType = this->context->GetFactory(thisBuyNode)->GetFactoryType();
    std::vector<int> all_buyNode_to_thisSellNode = this->context->GetGlobalFactoryTypeMap().at(this->context->GetFactory(thisSellNode)->GetFactoryType()).at(buyNodeType);
    
    // 设置机器人的任务路线
    this->context->GetRobot(robotID)->taskRoute_.push(thisBuyNode);
    this->context->GetRobot(robotID)->taskRoute_.push(thisSellNode);
    // 设置机器人的空闲状态
    this->context->GetRobot(robotID)->SetFlag(ROBOT_BUSY);

    // 把所有要去买的材料类型的节点到卖材料节点的路径锁住
    for(auto buyNodeID : all_buyNode_to_thisSellNode)
    {
        // this->globalGraph_[buyNodeID][thisSellNode] = this->INFINITE_;
        // this->globalGraph_[thisSellNode][buyNodeID] = this->INFINITE_;
        this->PreserveAndUpdateInfo(thisSellNode, buyNodeID, this->INFINITE_);
    } 
    // 把所有机器人到买材料的节点的路径锁住
    for(auto i = 0; i < this->context->GetRobotTotalNum(); ++i)
    {
        // this->globalGraph_[i][thisBuyNode] = this->INFINITE_;
        // this->globalGraph_[thisBuyNode][i] = this->INFINITE_;
        this->PreserveAndUpdateInfo(i, thisBuyNode, this->INFINITE_);
    }
    // 1. 根据机器人到卖家的最短路径route，分配一组买卖任务
    // 2. 分配任务后将相应路径的权值设为正无穷
}

void Distributor::CheckAllRobotsState()
{
    for(auto robotID = 0; robotID < this->context->GetRobotTotalNum(); ++robotID)
    {
        // 如果机器人处于任务状态
        if (this->context->GetRobot(robotID)->GetFlag() == ROBOT_BUSY)
        {
            if(!this->context->GetRobot(robotID)->taskRoute_.empty())
            {
                if (this->context->GetRobot(robotID)->GetNearbyFactoryId() == this->context->GetRobot(robotID)->taskRoute_.front());
                {   // 如果机器人到达了任务路线中的购买材料节点
                    this->context->GetRobot(robotID)->taskRoute_.pop(); // 移除第一个任务路线节点
                }
                
            }
        }
    }
}

/**
 * @brief 使用bellman_ford算法在图中寻找最短路径，并返回距离以及途径节点
 *
 * @param _graph_(现this->globalGraph_)  图
 * @param V                             图中节点的总数量
 * @param src                           出发节点
 * @param target                        目标节点
 * @return pair<double, vector<int>>    最短路径距离，途径节点序号
 */
std::pair<double, std::vector<int>> Distributor::BellmanFordRoute(int src, int target) {
    std::pair<double, std::vector<int>> _result_;
    if (src > 3){ //起始点不是机器人编号，打印错误信息
        std::cerr << "BellmanFord src ERROR, src: " << src << "; src should < 3" << std::endl;
        return _result_;
    }

    auto V = this->context->GetRobotTotalNum() + this->context->GetFactoryTotalNum();
    const int INF = this->INFINITE_; //将double转换为int???
    // 初始化距离数组和路径数组
    std::vector<int> dist(V, INF);
    std::vector<int> path(V, -1);
    dist[src] = 0;
    // 进行V-1轮松弛操作
    for (int i = 0; i < V - 1; i++) {
        for (int u = 0; u < V; u++) {
            for (int v = 0; v < V; v++) {
                if (this->globalGraph_[u][v] != INF && dist[u] != INF && dist[u] + this->globalGraph_[u][v] < dist[v]) {
                    dist[v] = dist[u] + this->globalGraph_[u][v];
                    path[v] = u; // 更新前驱节点
                }
            }
        }
    }
    // 检查负环
    for (int u = 0; u < V; u++) {
        for (int v = 0; v < V; v++) {
            if (this->globalGraph_[u][v] != INF && dist[u] != INF && dist[u] + this->globalGraph_[u][v] < dist[v]) {
                std::cerr << "该图存在负环，无法求解最短路径" << std::endl;
                return _result_;
            }
        }
    }
    // 输出最短路径距离
    for (int i = 0; i < V; i++) {
        if (i != target) continue;
        _result_.first = dist[i];
    }
    // 输出最短路径途径节点
    for (int i = 0; i < V; i++) {
        if (i != target) continue;
        std::stack<int> s;
        int j = i;
        // 从终点回溯到起点，把节点存储在stack中
        while (j != -1) {
            s.push(j);
            j = path[j];
        }
        // 从起点一个个弹出路经节点
        while (!s.empty()) {
            _result_.second.emplace_back(s.top());
            s.pop();
        }
    }
    return _result_;
}

// 根据传入的工厂id和类型，找到所有对应的权值（矩阵块），并返回对应目标工厂的id
std::vector<int> Distributor::FromIdTypeFindEdgeIndex(int factory_index, FactoryType to_factoryType) const {

    int factory_shift = this->context->GetRobotTotalNum(); // checked
    FactoryType current_type = this->context->GetFactory(factory_index)->GetFactoryType(); // checked

    std::vector<int> to_index = this->context->GetGlobalFactoryTypeMap().at(current_type).at(to_factoryType);

    for (int i = 0; i < to_index.size(); ++i) {
        to_index[i] += factory_shift;
    }

    return to_index;
};

std::vector<std::vector<double>> Distributor::DeepCopy2DVector(const std::vector<std::vector<double>>& orig) {
    std::vector<std::vector<double>> copy(orig.size(), std::vector<double>(orig[0].size()));
    for (size_t i = 0; i < orig.size(); i++) {
        for (size_t j = 0; j < orig[i].size(); j++) {
            copy[i][j] = orig[i][j];
        }
    }
    return copy;
    std::vector<int > a;
    a.reserve(0);
}

void Distributor::UpdateFromBroadcast() {
    // 对所有工厂-工厂按广播更新权值
    this->FFBroadcastUpdate();

    // 对所有机器人-工厂按广播更新权值
    for (int robot_index = 0; robot_index < this->context->GetRobotTotalNum(); ++robot_index) {
        RobotFlag current_robot_flag = this->context->GetRobot(robot_index)->GetFlag();

        // 对空闲的机器人按照目的地工厂是否有产品更新权值
        if (current_robot_flag == ROBOT_READY){
            for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
                RFBroadcastUpdate(robot_index, factory_index);
            }
        } else {
            // 对不空闲的机器人来说，不参与分配任务、不更新地图
            continue;
        }
    }
}

// 考虑到机器人分为BUSY和READY，这里传入机器人ID和工厂ID从而更新机器人到工厂的状态
void Distributor::RFBroadcastUpdate(int robot_index, int factory_index) {
    int factory_index_shift = factory_index + this->factoryIDShift_;

    // 无：false，有：true
    bool product_state = this->context->GetFactory(factory_index)->GetProductState();
    if (product_state){
//        double robot_x = this->context->GetRobot(robot_index)->GetCoordinate()[0];
//        double robot_y = this->context->GetRobot(robot_index)->GetCoordinate()[1];
//        double factory_x = this->context->GetFactory(robot_index)->GetCoordinate()[0];
//        double factory_y = this->context->GetFactory(robot_index)->GetCoordinate()[1];
//        double distance = sqrt(pow(factory_x - robot_x, 2) + pow(factory_y - robot_y, 2));

        double distance_coe = (this->distanceCoefficient_) * (this->context->DistanceFR(robot_index, factory_index));
        this->PreserveAndUpdateInfo(robot_index, factory_index_shift, distance_coe);
    } else {
        this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
    }
}

void Distributor::FFBroadcastUpdate() {
    // factory_index: 下游工厂
    for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
        FactoryClass current_class = this->context->GetFactory(factory_index)->GetFactoryClass();
        int edge_from_index = factory_index + this->factoryIDShift_;

        // 对4567类型工厂的仓库状态循环
        if (current_class == FactoryClass::B || current_class == FactoryClass::C){
            auto warehouse_state = this->context->GetFactory(factory_index)->GetWarehouseState();
            // 对该类型的每个仓库格类型循环
            for (auto warehouse_type : this->context->GetFactory(factory_index)->GetWarehouseType()) {
//                std::cerr << "1 是BC CLASS" << std::endl;
//                std::cerr << "工厂id: " << factory_index << " 现在仓库类型为: " << warehouse_type << std::endl;


                std::vector<int> edges_to = this->FromIdTypeFindEdgeIndex(factory_index, warehouse_type);
//                std::cerr << "找到的edges_to: ";
                for (auto item : edges_to) {
                    std::cerr << factory_index << " ";
                }
                std::cerr << std::endl;
                // 仓库状态为true，代表有货物，即为没有需求，正无穷
                bool no_need = !(warehouse_state[warehouse_type].first | warehouse_state[warehouse_type].second);
                if (no_need){
//                    std::cerr << "2 下游无需求" << std::endl;
                    for (auto edge_to_index : edges_to) {
                        this->PreserveAndUpdateInfo(edge_from_index, edge_to_index, this->INFINITE_);
                    }
                } else {
//                    std::cerr << "2 下游有需求" << std::endl;
                    // 下游有需求，对warehouse_type类的所有上游index循环
                    for (auto edge_to_index : edges_to ) {
                        std::cerr << edge_from_index << std::endl;
                        int up_index = edge_to_index - this->factoryIDShift_;
                        bool product_state = this->context->GetFactory(up_index)->GetProductState();
                        // 上游有产品
                        if (product_state){
//                            std::cerr << "3 上游有产品" << std::endl;
                            this->PreserveAndUpdateInfo(edge_from_index, edge_to_index, this->MAX_ENCOURAGE_);
                        } else {
//                            std::cerr << "3 上游无产品" << std::endl;
                            // 上游没有产品，根据是否在生产决定
                            FactoryFlag up_flag = this->context->GetFactory(up_index)->GetFactoryFlag();
                            // 在生产
                            if (up_flag == FactoryFlag::PRODUCING){
                                this->PreserveAndUpdateInfo(edge_from_index, edge_to_index, this->INFINITE_);
                            } else {
                                this->PreserveAndUpdateInfo(edge_from_index, edge_to_index, this->ZERO_);
                            }
                        }
                    }
                }
            }
        } else { // 工厂类型不为4567，本身不能当作下游，继续循环
            continue;
        }
    }
}

void Distributor::PreserveAndUpdateInfo(int u, int v, double value) {
    double preserve = this->globalGraph_[u][v];
    this->historyGraph_[u][v] = preserve;
    this->historyGraph_[v][u] = preserve;

    this->globalGraph_[u][v] = value;
    this->globalGraph_[v][u] = value;
}

void Distributor::ExtractInfo(int u, int v) {
    double value = this->historyGraph_[u][v];
    this->globalGraph_[u][v] = value;
    this->globalGraph_[v][u] = value;
}
