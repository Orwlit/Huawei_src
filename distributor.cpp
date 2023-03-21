//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <cmath>

#include "distributor.h"


Distributor::Distributor() {
    std::shared_ptr<Context> sharedPtr = std::make_shared<Context>();
    this->context = sharedPtr;


    std::cout << "OK" << std::flush; // 准备就绪，开始答题
}

bool Distributor::run() {
    while (this->context->UpdateAllStatus()){

        //开始与控制台交互
        std::cout << this->context->GetFrameId() << std::endl;



//        if (this->context->GetRobot(0)->GetLinearVelocity() < 0.1){
//            std::cerr << "Robot 0 STOPPED!!!" << std::endl;
//        }
//        if (this->context->GetRobot(1)->GetLinearVelocity() < 0.1){
//            std::cerr << "Robot 1 STOPPED!!!" << std::endl;
//        }
//        this->context->GetRobot(2)->HighSpeedMove(25.0f, 25.0f, this->context->GetDt());
//        this->context->GetRobot(3)->HighSpeedMove(25.0f, 25.0f, this->context->GetDt());


//        if (!(this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(1), 8.5))){
//            std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
//        }

        std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
        bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(3), 15);
        if (aboutToCrash){
            this->context->GetRobot(0)->Rotate(M_PI);
            this->context->GetRobot(3)->Rotate(M_PI);
        }else {
            this->context->GetRobot(0)->HighSpeedMove(40.0f, 40.0f, this->context->GetDt());
            this->context->GetRobot(3)->HighSpeedMove(10.0f, 40.0f, this->context->GetDt());
        }
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(i), this->context->GetRobot(j), 9);
//                if (aboutToCrash){
//                    this->context->GetRobot(i)->Rotate(M_PI/2);
//                    this->context->GetRobot(j)->Rotate(M_PI/2);
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


std::vector<double> Distributor::BellmanFordDistance(std::vector<std::vector<int>>& graph, int V, int src, int target) {
    // 初始化距离数组和路径数组
    std::vector<double> dist(V, this->INFINITE_);
    std::vector<int> path(V, -1);
    dist[src] = 0;
    std::vector<double> route;

    // 进行V-1轮松弛操作
    for (int i = 0; i < V - 1; i++) {
        for (int u = 0; u < V; u++) {
            for (int v = 0; v < V; v++) {
                if (graph[u][v] != this->INFINITE_ && dist[u] != this->INFINITE_ && dist[u] + graph[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph[u][v];
                    path[v] = u; // 更新前驱节点
                }
            }
        }
    }

    // 检查负环
    for (int u = 0; u < V; u++) {
        for (int v = 0; v < V; v++) {
            if (graph[u][v] != this->INFINITE_ && dist[u] != this->INFINITE_ && dist[u] + graph[u][v] < dist[v]) {
                std::cerr << "该图存在负环，无法求解最短路径" << std::endl;
                return route;
            }
        }
    }

    // 输出最短路径
    std::cerr << "从源节点 " << src << " 到目标节点 " << target << std::endl;
    for (int i = 0; i < V; i++) {
        if (i != target) continue;
        std::cerr << "最短距离为：" << dist[i] << std::endl;
    }
    std::cerr << "最短路径为：";
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
            route.push_back(s.top());
//            std::cerr << s.top();
//            if (s.top() != i) {
//                std::cerr << " -> ";
//            }
            s.pop();
        }
    }
    return route;
}

void Distributor::DistributeTask(std::vector<int> route) {
    //TODO: 这个函数完成两个功能:
    // 1. 根据机器人到卖家的最短路径route，分配一组买卖任务
    // 2. 分配任务后将相应路径的权值设为正无穷
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


