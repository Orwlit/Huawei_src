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

//        std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;

        if (this->context->GetFrameId() == 1){
            this->context->PrintHistoryMap(this->context->GetInitialHistoryGraph(), "最开始");
        }

        // 根据广播更新地图
        this->UpdateNeed();

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


        if (this->context->GetFrameId() > 50){

            for (int ready_robot_index = 0; ready_robot_index < this->context->GetRobotTotalNum(); ++ready_robot_index) {
                if (this->context->GetRobot(ready_robot_index)->GetFlag() == ROBOT_READY){



                    double route_value = this->INFINITE_;
                    std::pair<double, std::vector<int>> route_best;
                    for (int seller_index : this->context->sellersNode_) {
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



//                    this->context->PrintHistoryMap(this->globalGraph_, "222globalGraph_");
//                    this->context->PrintHistoryMap(this->historyGraph_, "222historyGraph_");

                    std::cerr << "NOT PASS" << std::endl;
                    this->DistributeTask(route_best);
                    std::cerr << "PASS" << std::endl;

                    this->UpdateNeed();




                } else {
                    continue;
                }
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

    int thisBuy_index = thisBuyNode - this->factoryIDShift_;
    int thisSell_index = thisSellNode - this->factoryIDShift_;

    std::cerr << "route: ";
    for (auto item : route.second) {
        std::cerr << item << " ";
    }
    std::cerr << std::endl;

//    std::cerr << "thisBuy_index: " << thisBuy_index << " thisSell_index: " << thisSell_index << std::endl;


//    std::cerr << "1 PASS" << std::endl;


    FactoryType sellIndexType = this->context->GetFactory(thisSell_index)->GetFactoryType();
    FactoryType buyIndexType = this->context->GetFactory(thisBuy_index)->GetFactoryType();

//    std::cerr << "2 PASS" << std::endl;


    // 设置机器人的任务路线
    this->context->GetRobot(robotID)->task_Buy_Sell_.first = thisBuy_index;
    this->context->GetRobot(robotID)->task_Buy_Sell_.second = thisSell_index;
    // 设置机器人的空闲状态
    this->context->GetRobot(robotID)->SetFlag(ROBOT_BUSY);

//    std::cerr << "3 PASS" << std::endl;



    // 对于8和9类型的工作台，收购需求一直打开
    // !特殊情况
    if (this->context->GetFactory(thisSell_index)->GetFactoryClass() == D)
    {



        int buy_index = this->context->GetRobot(robotID)->task_Buy_Sell_.first;
        int sell_index = this->context->GetRobot(robotID)->task_Buy_Sell_.second;

        this->context->GetFactory(buy_index)->SetProductFlag(true);  // 设置购买节点生产的产品以及被预售光了
        this->context->GetFactory(sell_index)->SetWarehouseFlag(buyIndexType, false);

    }
    else
    {

        std::cerr << "4 PASS" << std::endl;


        //buyNodeType = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.first)->GetFactoryType();  // 获得机器人购买节点的类型
        int buy_index = this->context->GetRobot(robotID)->task_Buy_Sell_.first;
        int sell_index = this->context->GetRobot(robotID)->task_Buy_Sell_.second;

        std::cerr << "5 PASS" << std::endl;


        // FactoryType sellNodeType = this->context->GetFactory(this->context->GetRobot(robotID)->task_Buy_Sell_.second)->GetFactoryType();  // 获得机器人售卖节点的类型
        this->context->GetFactory(buy_index)->SetProductFlag(true);  // 设置购买节点生产的产品以及被预售光了
        this->context->GetFactory(sell_index)->SetWarehouseFlag(buyIndexType, true);  // 设置售卖节点的收购材料正在送货

        std::cerr << "6 PASS" << std::endl;

    }
    // 1. 根据机器人到卖家的最短路径route，分配一组买卖任务
    // 2. 分配任务后将相应路径的权值设为正无穷

//    if (thisSell_index == 12){
//        std::cerr << "RobotID: " << robotID << " Buy type: " << buyIndexType << " Sell type: " << sellIndexType << std::endl;
//        std::cerr << "Buy product state: " << this->context->GetFactory(thisBuy_index)->GetProductState() << " Buy product flag: " << this->context->GetFactory(thisBuy_index)->GetProductFlag() <<std::endl;
//        for (auto item : this->context->GetFactory(thisSell_index)->GetWarehouseState()) {
//            std::cerr << "Sell Warehouse Type: " << item.first << " State: " << item.second.first << " Flag: " << item.second.first <<std::endl;
//        }
//        std::cerr << "------------------------------------" << std::endl;
//    }

}

// 机器人状态检查以及运动任务分配
void Distributor::CheckAllRobotsState()
{
    for(auto robotID = 0; robotID < this->context->GetRobotTotalNum(); ++robotID)
    {   
        // 如果机器人处于任务状态
        if (this->context->GetRobot(robotID)->GetFlag() == ROBOT_BUSY)
        {   
            int& buyIndex = this->context->GetRobot(robotID)->task_Buy_Sell_.first;
            int& sellIndex = this->context->GetRobot(robotID)->task_Buy_Sell_.second;
            if (buyIndex != -1)  // 说明机器人已经被分配了购买材料任务
            {
                int carryingType = this->context->GetRobot(robotID)->GetCarryingType();
                FactoryType buyType = this->context->GetFactory(buyIndex)->GetFactoryType();
                bool isNearBy_buy = this->context->GetRobot(robotID)->Buy(buyIndex);
                // 如果机器人完成了购买材料任务
                if (isNearBy_buy)
                {
                    FactoryType buyNodeType = this->context->GetFactory(buyIndex)->GetFactoryType();  // 获得机器人购买节点的类型
                    this->context->GetFactory(buyIndex)->SetProductFlag(false);  // 购买完成，解锁工作台的预售状态
                    buyIndex = -1; // 如果机器人购买完成则设置购买节点位-1，标记完成了材料购买
                    continue; // 跳帧，等待下一帧检索进入卖材料任务 
                }
                // 机器人仍然处于购买材料任务
                else
                {
                    double buyNode_x = this->context->GetFactory(buyIndex)->GetCoordinate()[0];
                    double buyNode_y = this->context->GetFactory(buyIndex)->GetCoordinate()[1];
                    this->context->GetRobot(robotID)->curTarget_ = std::make_pair(buyNode_x, buyNode_y); // 设置机器人的目标移动点
                    
                    // FactoryType buyNodeType = this->context->GetFactory(buyIndex)->GetFactoryType();  // 获得机器人购买节点的类型
                    // this->context->GetFactory(buyIndex)->SetProductFlag(true);  // 设置购买节点的购买物品位置已经被预定售卖

                    // FactoryType sellNodeType = this->context->GetFactory(sellIndex)->GetFactoryType();  // 获得机器人售卖节点的类型
                    // this->context->GetFactory(sellIndex)->SetWarehouseFlag(buyNodeType, true);  // 设置售卖节点的收购材料正在送货
                }
            }
            else if (sellIndex != -1)  // 说明机器人已经被分配了卖材料任务
            {   
                // 如果机器人完成了销售材料任务
                FactoryType carryingType = static_cast<FactoryType>(this->context->GetRobot(robotID)->GetCarryingType());
                bool isNearBy_sell = this->context->GetRobot(robotID)->Sell(sellIndex);
                if ( isNearBy_sell )
                {
                    this->context->GetFactory(sellIndex)->SetWarehouseFlag(carryingType, false);  // 设置售卖节点的收购完成
                    sellIndex = -1; // 如果机器人购买完成则设置购买节点位-1，标记完成了材料购买
                    this->context->GetRobot(robotID)->SetFlag(ROBOT_READY);  // 机器人完成购买回归空闲状态
                    continue; // 跳帧，等待下一帧检索和分配任务
                }
                // 机器人仍然处于销售材料任务
                else
                {
                    double sellNode_x = this->context->GetFactory(sellIndex)->GetCoordinate()[0];
                    double sellNode_y = this->context->GetFactory(sellIndex)->GetCoordinate()[1];
                    this->context->GetRobot(robotID)->curTarget_ = std::make_pair(sellNode_x, sellNode_y); // 设置机器人的目标移动点
                }
            }

            // 根据上面机器人处于购买材料和售卖材料阶段对机器人设置的目标点控制机器人移动
            double destination_x =  this->context->GetRobot(robotID)->curTarget_.first;
            double destination_y = this->context->GetRobot(robotID)->curTarget_.second;
            this->context->GetRobot(robotID)->HighSpeedMove(destination_x, destination_y, this->context->GetDt());
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

    int factory_shift = this->context->GetRobotTotalNum();
    FactoryType current_type = this->context->GetFactory(factory_index)->GetFactoryType();

//    std::cerr << "NOT PASS" << std::endl;
//    std::cerr << "factory_index: " << factory_index << " to_factoryType: " << to_factoryType << std::endl;
    std::vector<int> nodes_to = this->context->GetGlobalFactoryTypeMap().at(current_type).at(to_factoryType);
//    std::cerr << "PASS" << std::endl;

    for (int i = 0; i < nodes_to.size(); ++i) {
        nodes_to[i] += factory_shift;
    }

    return nodes_to;
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

void Distributor::UpdateNeed() {
    // 1. 对所有工厂-工厂按广播更新权值
//    std::cerr << "1 NOT PASS" << std::endl;
    this->FFNeedUpdate();
//    std::cerr << "1 PASS" << std::endl;

//    std::cerr << "2 NOT PASS" << std::endl;
    this->RFHaveNeedUpdate();
//    std::cerr << "2 PASS" << std::endl;

    // 2. 机器人-工厂权值更新
//    std::cerr << "3 NOT PASS" << std::endl;
    this->RFHaveProductUpdate();
//    std::cerr << "3 PASS" << std::endl;
}

// 按是否耽误取货更新机器人-工厂权值
void Distributor::RFHaveProductUpdate() {
    const double dt = this->context->GetDt();

    for (int robot_index = 0; robot_index < this->context->GetRobotTotalNum(); ++robot_index) {
        for (int factory_index = 0; factory_index < this->context->GetFactoryTotalNum(); ++factory_index) {
            const int factory_index_shift = factory_index + this->factoryIDShift_;
            double distance = this->context->DistanceFR(robot_index, factory_index);
            double distance_coe = this->distanceCoefficient_ * distance;

            // 1层，有无派送？
            bool have_delivery = this->context->GetFactory(factory_index)->GetProductFlag();
            if (have_delivery){
                this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
            } else {
                // product_state 无产品：false，有产品：true
                bool ok_to_ignore_product_state = false;
                bool product_state = this->context->GetFactory(factory_index)->GetProductState();
                int remaining_frame = this->context->GetFactory(factory_index)->GetRemainingFrame();

                if (remaining_frame == -1){ // 没在生产，需要根据产品状态决定
                    ok_to_ignore_product_state = false;
                } else {
                    double distance_charge_to_factory = this->chargeVelocity_ * dt * remaining_frame;
                    ok_to_ignore_product_state = distance > distance_charge_to_factory;
                }


//                // 2层，耽不耽误拿产品？？
//                if (product_state || ok_to_ignore_product_state){
//                    this->PreserveAndUpdateInfo(robot_index, factory_index_shift, distance_coe);
//                } else {
//                    this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
//                }

                // 2层，耽不耽误拿产品？？
                if (product_state){
                    this->PreserveAndUpdateInfo(robot_index, factory_index_shift, distance_coe);
                } else {
                    this->PreserveAndUpdateInfo(robot_index, factory_index_shift, this->INFINITE_);
                }

            }
        }
    }
}



void Distributor::FFNeedUpdate() {
    // factory_index: 下游工厂
    for (int factory_from_index = 0; factory_from_index < this->context->GetFactoryTotalNum(); ++factory_from_index) {
        FactoryClass current_class = this->context->GetFactory(factory_from_index)->GetFactoryClass();
        int node_from = factory_from_index + this->factoryIDShift_;

        // 对4567类型工厂的仓库状态循环
        if (current_class == FactoryClass::B || current_class == FactoryClass::C){
            auto warehouse_state = this->context->GetFactory(factory_from_index)->GetWarehouseState();
            // 对该类型的每个仓库格类型循环
            for (auto warehouse_type : this->context->GetFactory(factory_from_index)->GetWarehouseType()) {
//                std::cerr << "1 是BC CLASS" << std::endl;
//                std::cerr << "工厂id: " << factory_from_index << " 现在仓库类型为: " << warehouse_type << std::endl;


                std::vector<int> nodes_to = this->FromIdTypeFindEdgeIndex(factory_from_index, warehouse_type);
//                std::cerr << "找到的edges_to: ";
//                for (auto item : nodes_to) {
//                    std::cerr << factory_from_index << " ";
//                }
//                std::cerr << std::endl;

                // 有需求的定义：仓库里没货，且没人给送货
                bool have_need = (!(warehouse_state[warehouse_type].first)) && (!(warehouse_state[warehouse_type].second));
                if (!have_need){
//                    std::cerr << "2 下游无需求" << std::endl;
                    for (auto node_to : nodes_to) {
                        this->PreserveAndUpdateInfo(node_from, node_to, this->INFINITE_);
                    }
                } else {
//                    std::cerr << "2 下游有需求" << std::endl;

                    // 下游有需求，对warehouse_type类的所有上游index循环
                    for (auto node_to : nodes_to ) {
                        int factory_to_index = node_to - this->factoryIDShift_;
                        double distance = this->context->DistanceFF(factory_from_index, factory_to_index);
                        double distance_zero = distance * this->ZERO_;
                        double distance_max_encourage = distance * this->MAX_ENCOURAGE_;

//                        std::cerr << node_from << std::endl;
                        bool product_state = this->context->GetFactory(factory_to_index)->GetProductState();
                        // 上游有产品
                        if (product_state){
//                            std::cerr << "3 上游有产品" << std::endl;
                            this->PreserveAndUpdateInfo(node_from, node_to, distance_max_encourage);
                        } else {
//                            std::cerr << "3 上游无产品" << std::endl;

                            // 在生产
                            bool producing = this->context->GetFactory(factory_to_index)->GetRemainingFrame() > -1;
                            if (producing){
                                this->PreserveAndUpdateInfo(node_from, node_to, this->INFINITE_);
                            } else {
                                this->PreserveAndUpdateInfo(node_from, node_to, distance_zero);
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

//                std::cerr << "NOT PASS" << std::endl;
//                std::cerr << "factory_index: " << factory_index << std::endl;
//                std::cerr << this->context->GetFactory(factory_index)->GetWarehouseState().size() << std::endl;
                bool warehouse_flag = this->context->GetFactory(factory_index)->GetWarehouseState().at(UNKNOWN).second;
//                std::cerr << "PASS" << std::endl;


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
