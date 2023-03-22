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

    std::cout << "OK" << std::flush; // 准备就绪，开始答题
}

bool Distributor::run() {
    while (this->context->UpdateAllStatus()){

        //开始与控制台交互
        std::cout << this->context->GetFrameId() << std::endl;


        this->context->GetRobot(0)->HighSpeedMove(40.0f, 40.0f, this->context->GetDt());
        this->context->GetRobot(3)->HighSpeedMove(10.0f, 40.0f, this->context->GetDt());


//        if (!(this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(1), 8.5))){
//            std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
//        }

        std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
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
                if ()
                {
                    /* code */
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
    int factory_shift = this->context->GetRobotTotalNum();
    FactoryType current_type = this->context->GetFactory(factory_index)->GetFactoryType();
    std::vector<int> to_index = this->context->GetGlobalFactoryTypeMap().at(current_type).at(to_factoryType);

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
}

void Distributor::UpdateFromBroadcast() {

}

void Distributor::RFBroadcastUpdate(int robot_index, int factory_index) {
    // 无：0，有：1
    int product_state = this->context->GetFactory(factory_index)->GetProductState();
    if (product_state == 1){
        double robot_x = this->context->GetRobot(robot_index)->GetCoordinate()[0];
        double robot_y = this->context->GetRobot(robot_index)->GetCoordinate()[1];
        double factory_x = this->context->GetFactory(robot_index)->GetCoordinate()[0];
        double factory_y = this->context->GetFactory(robot_index)->GetCoordinate()[1];
        double distance = sqrt(pow(factory_x - robot_x, 2) + pow(factory_y - robot_y, 2));

        // 是否需要保存历史数据???
//        this->historyGraph_[robot_index][factory_index + this->factoryIDShift_] = this->globalGraph_[robot_index][factory_index + this->factoryIDShift_];
//        this->historyGraph_[factory_index + this->factoryIDShift_][robot_index] = this->globalGraph_[factory_index + this->factoryIDShift_][robot_index];
        // 更新权值
        this->globalGraph_[robot_index][factory_index + this->factoryIDShift_] = distance;
        this->globalGraph_[factory_index + this->factoryIDShift_][robot_index] = distance;
    } else if (product_state == 0){
        // 保存历史数据
        this->historyGraph_[robot_index][factory_index + this->factoryIDShift_] = this->globalGraph_[robot_index][factory_index + this->factoryIDShift_];
        this->historyGraph_[factory_index + this->factoryIDShift_][robot_index] = this->globalGraph_[factory_index + this->factoryIDShift_][robot_index];
        // 更新权值
        this->globalGraph_[robot_index][factory_index + this->factoryIDShift_] = this->INFINITE_;
        this->globalGraph_[factory_index + this->factoryIDShift_][robot_index] = this->INFINITE_;
    }
}

void Distributor::FFBroadcastUpdate(int up_index, int down_index) {
    for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
        FactoryClass current_class = this->context->GetFactory(factory_index)->GetFactoryClass();
        // 对4567类型工厂的仓库状态循环
        if (current_class == FactoryClass::B || current_class == FactoryClass::C){
            auto warehouse_state = this->context->GetFactory(factory_index)->GetWarehouseState();
            for (auto warehouse_type : this->context->GetFactory(factory_index)->GetWarehouseType()) {
                if (warehouse_state[warehouse_type]){ // 仓库状态为true，代表有货物，即为没有需求
                    //TODO: 根据是否在生产决定权值
                } else {
                    //TODO: 有需求，负无穷
                }
            }
        }
    }


    FactoryType up_type = this->context->GetFactory(up_index)->GetFactoryType();
    FactoryType down_type = this->context->GetFactory(down_index)->GetFactoryType();
    auto down_warehouse_state = this->context->GetFactory(down_index)->GetWarehouseState();
    auto down_warehouse_type = this->context->GetFactory(down_index)->GetWarehouseType();
    FactoryFlag down_demand;


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
