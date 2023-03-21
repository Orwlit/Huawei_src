//
// Created by orw on 3/14/23.
//

#ifndef HUAWEI_DISTRIBUTOR_H
#define HUAWEI_DISTRIBUTOR_H
#pragma once
#include <climits>
#include <stack>

#include "context.h"
#include "robot.h"
#include "factory.h"

//enum DistributorFlag{
//    AIMLESS = 0,
//    TRICK = 1,
//    ELITE_456 = 2,
//    GLOBAL_456 = 3
//};

//enum Priority{
//    ANYWAY = 0,
//    ONE = 1,
//    TWO = 2,
//    THREE = 3,
//    FOUR = 4,
//    FIVE = 5
//};

class Distributor{
public:
    Distributor();

    //Basic Function
    bool MaintainGraph(); //从context中维护图结构
    std::pair<double, std::vector<int>> BellmanFordRoute(int src, int target);
    std::vector<double> BellmanFordDistance(std::vector<std::vector<int>>& graph, int V, int src, int target);

    //全局图优化，利用A层到D层最短距离，结合机器人距离派发任务。(A层：123；B层：456；C层：7；D层：89)
    bool GraphOptimization();

    //Tasks
    void DistributeTask(std::vector<int> route);

    bool run();
private:
    std::shared_ptr<Context> context;
    std::vector<std::vector<double>> globalGraph_;
    std::vector<std::vector<double>> historyGraph_;


    const double DISCONNECT_ = 0.0;
    const double INFINITE_ = 999.0; // 表示无穷大

};
#endif //HUAWEI_DISTRIBUTOR_H
