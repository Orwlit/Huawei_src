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

        if (this->context->GetFrameId() == 1){
            this->context->PrintHistoryMap(this->context->GetInitialHistoryGraph(), "最开始");
        }

        // 根据广播更新地图
        this->UpdateFromBroadcast();

//        for (int ready_robot_index = 0; ready_robot_index < this->context->GetRobotTotalNum(); ++ready_robot_index) {
//            if (this->context->GetRobot(ready_robot_index)->GetFlag() == ROBOT_READY){
//                for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
//                    if (this->context->GetFactory(factory_index)->GetFactoryClass() == D){
//                        continue;
//                    } else {
//                        this->RFHaveProductUpdate(ready_robot_index, factory_index);
//                    }
//                }
//            } else {
//                continue;
//            }
//        }




//        if (this->context->GetFrameId() < 3){
//            this->context->PrintHistoryMap(this->globalGraph_, "globalGraph_");
//            this->context->PrintHistoryMap(this->historyGraph_, "historyGraph_");
//            std::cerr << std::endl << std::endl;
//        }

        //开始与控制台交互
        std::cout << this->context->GetFrameId() << std::endl;

        for (int ready_robot_index = 0; ready_robot_index < this->context->GetRobotTotalNum(); ++ready_robot_index) {
            if (this->context->GetRobot(ready_robot_index)->GetFlag() == ROBOT_READY){


                double route_value = this->INFINITE_;
                std::pair<double, std::vector<int>> route_best;
                for (int seller_index : this->context->sellers_) {
                    int seller_shift_index = seller_index + this->factoryIDShift_;
                    auto route_tmp = this->BellmanFordRoute(ready_robot_index, seller_index);
                    if (route_value > route_tmp.first){
                        route_value = route_tmp.first;
                        route_best = route_tmp;
                    }
                }



//                std::cerr << "route_value: " << route_value << "route_best: " << std::endl;
//                for (auto route : route_best.second) {
//                    std::cerr << route << " ";
//                }
//                std::cerr << std::endl << std::endl;



                this->DistributeTask(route_best);
                this->UpdateFromBroadcast();


//                this->context->PrintHistoryMap(this->globalGraph_, "222globalGraph_");
//                this->context->PrintHistoryMap(this->historyGraph_, "222historyGraph_");


            } else {
                continue;
            }
        }

        this->CheckAllRobotsState();


        // 两两检查是否相撞，共6次
//        for (int i = 0; i < 4; ++i) {
//            for (int j = i + 1; j < 4; ++j) {
//                bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(i), this->context->GetRobot(j), 18);
//                if (aboutToCrash){
//                    this->context->GetRobot(i)->Rotate(M_PI);
//                    this->context->GetRobot(j)->Rotate(M_PI);
////                    this->context->GetRobot(i)->Forward(0);
//                }
//            }
//        }



        //与控制台交互结束
        std::cout << "OK" << std::flush;

        this->context->SetPreviousFrameId(this->context->GetFrameId());
    }
    return false;
}


bool Distributor::GraphOptimization() {


    return false;
}

// 分配
void Distributor::DistributeTask(std::pair<double, std::vector<int>>& route) 
{
    int robotID = route.second[0];  // 机器人ID
    int thisBuyNode = route.second[1];  // 要去购买东西的节点ID
    int thisSellNode = route.second[2]; // 要去卖东西的节点ID

    int thisBuy_index = route.second[1] - this->factoryIDShift_;  //
    int thisSell_index = route.second[2] - this->factoryIDShift_; //

    FactoryType sellNodeType = this->context->GetFactory(thisSell_index)->GetFactoryType();
    FactoryType buyNodeType = this->context->GetFactory(thisBuy_index)->GetFactoryType();
   
    // 设置机器人的任务路线
    this->context->GetRobot(robotID)->task_Buy_Sell_.first = thisBuy_index;
    this->context->GetRobot(robotID)->task_Buy_Sell_.second = thisSell_index;
    // 设置机器人的空闲状态
    this->context->GetRobot(robotID)->SetFlag(ROBOT_BUSY);

    // 对于8和9类型的工作台，收购需求一直打开
    // !特殊情况
    if (this->context->GetFactory(thisSell_index)->GetFactoryClass() == D)
    {
        this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->SetWarehouseFlag(sellNodeType, false); 
    }
    else
    {
         //buyNodeType = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->GetFactoryType();  // 获得机器人购买节点的类型
        this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->SetWarehouseFlag(buyNodeType, true);  // 设置购买节点的购买物品位置已经被预定售卖
    
        // FactoryType sellNodeType = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->GetFactoryType();  // 获得机器人售卖节点的类型
        this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->SetWarehouseFlag(sellNodeType, true);  // 设置售卖节点的收购材料正在送货
    }
    // 1. 根据机器人到卖家的最短路径route，分配一组买卖任务
    // 2. 分配任务后将相应路径的权值设为正无穷
}

// 机器人状态检查以及运动任务分配
void Distributor::CheckAllRobotsState()
{
    for(auto robotID = 0; robotID < this->context->GetRobotTotalNum(); ++robotID)
    {   
        // 如果机器人处于任务状态
        if (this->context->GetRobot(robotID)->GetFlag() == ROBOT_BUSY)
        {
            if (this->context->GetRobot(robotID)->task_Buy_Sell_.first != -1)  // 说明机器人已经被分配了购买材料任务
            {   
                // 如果机器人完成了购买材料任务
                if (this->context->GetRobot(robotID)->Buy(this->context->GetRobot(robotID)->task_Buy_Sell_.first))
                {
                    this->context->GetRobot(robotID)->task_Buy_Sell_.first = -1; // 如果机器人购买完成则设置购买节点位-1，标记完成了材料购买
                    continue; // 跳帧，等待下一帧检索进入卖材料任务 
                }
                // 机器人仍然处于购买材料任务
                else
                {
                    double buyNode_x = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->GetCoordinate()[0];
                    double buyNode_y = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->GetCoordinate()[1];
                    this->context->GetRobot(robotID)->curTarget_ = std::make_pair(buyNode_x, buyNode_y); // 设置机器人的目标移动点
                    
                    FactoryType buyNodeType = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->GetFactoryType();  // 获得机器人购买节点的类型
                    this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->SetWarehouseFlag(buyNodeType, true);  // 设置购买节点的购买物品位置已经被预定售卖

                    FactoryType sellNodeType = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->GetFactoryType();  // 获得机器人售卖节点的类型
                    this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->SetWarehouseFlag(sellNodeType, true);  // 设置售卖节点的收购材料正在送货
                }
            }
            else if (this->context->GetRobot(robotID)->task_Buy_Sell_.second != -1)  // 说明机器人已经被分配了卖材料任务
            {   
                // 如果机器人完成了销售材料任务
                if (this->context->GetRobot(robotID)->Sell(this->context->GetRobot(robotID)->task_Buy_Sell_.second))
                {
                    this->context->GetRobot(robotID)->task_Buy_Sell_.second = -1; // 如果机器人购买完成则设置购买节点位-1，标记完成了材料购买
                    this->context->GetRobot(robotID)->SetFlag(ROBOT_READY);  // 机器人完成购买回归空闲状态
                    continue; // 跳帧，等待下一帧检索和分配任务
                }
                // 机器人仍然处于销售材料任务
                else
                {
                    double sellNode_x = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->GetCoordinate()[0];
                    double sellNode_y = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->GetCoordinate()[1];
                    this->context->GetRobot(robotID)->curTarget_ = std::make_pair(sellNode_x, sellNode_y); // 设置机器人的目标移动点
                }
            }

            // 根据上面机器人处于购买材料和售卖材料阶段对机器人设置的目标点控制机器人移动
            this->context->GetRobot(robotID)->HighSpeedMove(this->context->GetRobot(robotID)->curTarget_.first, this->context->GetRobot(robotID)->curTarget_.second, this->context->GetDt());
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
    const double INF = this->INFINITE_; //将double转换为int???
    // 初始化距离数组和路径数组
    std::vector<double> dist(V, INF);
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

//    std::cerr << "NOT PASS" << std::endl;
//    std::cerr << "factory_index: " << factory_index << " to_factoryType: " << to_factoryType << std::endl;
    std::vector<int> to_index = this->context->GetGlobalFactoryTypeMap().at(current_type).at(to_factoryType);
//    std::cerr << "NOT PASS" << std::endl;

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
    // 1. 对所有工厂-工厂按广播更新权值
    this->FFNeedUpdate();
    this->RFHaveNeedUpdate();

    // 2. 机器人-工厂权值更新
    this->RFHaveProductUpdate();


}

// 按是否耽误取货更新机器人-工厂权值
void Distributor::RFHaveProductUpdate() {
    const double dt = this->context->GetDt();

    for (int robot_index = 0; robot_index < this->context->GetRobotTotalNum(); ++robot_index) {
        for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
            const int factory_index_shift = factory_index + this->factoryIDShift_;

            // product_state 无产品：false，有产品：true
            bool ok_to_ignore_product_state = false;
            bool product_state = this->context->GetFactory(factory_index)->GetProductState();
            int remaining_frame = this->context->GetFactory(factory_index)->GetRemainingFrame();

            if (remaining_frame == -1){ // 没在生产，需要根据产品状态决定
                ok_to_ignore_product_state = false;
            } else {
                double distance_charge_to_factory = this->chargeVelocity_ * dt * remaining_frame;
                double distance = this->context->DistanceFR(robot_index, factory_index);
                ok_to_ignore_product_state = distance > distance_charge_to_factory;
            }


            if (product_state || ok_to_ignore_product_state){
                double distance_coe = (this->distanceCoefficient_) * (this->context->DistanceFR(robot_index, factory_index));
                this->PreserveAndUpdateInfo(robot_index, factory_index_shift, distance_coe);
            } else {
                this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
            }
        }
    }
}



void Distributor::FFNeedUpdate() {
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
//                for (auto item : edges_to) {
//                    std::cerr << factory_index << " ";
//                }
//                std::cerr << std::endl;
                // 仓库状态为true，代表有货物，即为没有需求，正无穷
                bool no_need = !(warehouse_state[warehouse_type].first) && !(warehouse_state[warehouse_type].second);

                if (no_need){
//                    std::cerr << "2 下游无需求" << std::endl;
                    for (auto edge_to_index : edges_to) {
                        this->PreserveAndUpdateInfo(edge_from_index, edge_to_index, this->MAX_ENCOURAGE_); //TODO: BUG
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
                                this->PreserveAndUpdateInfo(edge_from_index, edge_to_index, this->INFINITE_); //TODO: BUG
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

// 机器人作为工厂上游，按需求对A、D类更新
void Distributor::RFHaveNeedUpdate() {
    for (int robot_index = 0; robot_index < this->context->GetRobotTotalNum(); ++robot_index) {
        for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
            const int factory_index_shift = factory_index + this->factoryIDShift_;
            FactoryClass current_class = this->context->GetFactory(factory_index)->GetFactoryClass();

            if (current_class == FactoryClass::A){
                // warehouse_flag: true: delivering, false: not delivering
                auto print = this->context->GetFactory(factory_index)->GetWarehouseState();
                bool warehouse_flag = this->context->GetFactory(factory_index)->GetWarehouseState().at(UNKNOWN).second;
                if (warehouse_flag){
                    this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
                } else {
                    double distance = this->context->DistanceFR(robot_index, factory_index);
                    double distance_coe = distance * this->distanceCoefficient_;
                    this->PreserveAndUpdateInfo(robot_index, factory_index_shift, distance_coe);
                }
            }
            if (current_class == FactoryClass::D){
//                std::cerr << "FactoryID: " << factory_index << "\tD" << std::endl;
                this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
            } else {
                continue;
            }
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
