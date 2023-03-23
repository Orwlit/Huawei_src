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
    std::pair<double, std::vector<int>> BellmanFordRoute(int src, int target);
    void PreserveAndUpdateInfo(int u, int v, double value); // 将u，v坐标的数据保存到历史地图
    void ExtractInfo(int u, int v); // 将u，v坐标的数据从历史地图读出

    // 分配逻辑
    bool GraphOptimization(); // 全局图优化，利用A层到D层最短距离，结合机器人距离派发任务。(A层：123；B层：456；C层：7；D层：89)
    void UpdateFromBroadcast(); // 通过每帧信息更新权值
    void UpdateFromPlanning(); // 根据机器人寻路更新权值
    void UpdateFromTask(); // 根据分配任务情况更新权值

    // 循环所有工厂，按广播更新
    void FFNeedUpdate();
    void RFHaveNeedUpdate();
    // 考虑到机器人分为BUSY和READY，这里传入机器人ID和工厂ID从而更新机器人到工厂的状态
    void RFHaveProductUpdate();// 机器人广播更新，根据传入的机器人编号和工厂编号，对工厂是否有产品更新权值

    //Tasks
    void DistributeTask(std::pair<double, std::vector<int>>& route);
    void CheckAllRobotsState();
    // 查找工具
    [[nodiscard]] std::vector<int> FromIdTypeFindEdgeIndex(int factory_index, FactoryType to_factoryType) const;
    std::vector<std::vector<double>> DeepCopy2DVector(const std::vector<std::vector<double>>& orig);

    bool run();
private:
    std::shared_ptr<Context> context;
    std::vector<std::vector<double>> globalGraph_;
    std::vector<std::vector<double>> historyGraph_;
    std::map<int, std::map<FactoryType, std::vector<double*>>> idTypeEdge_; // 依次按工厂id、工厂类型type进行索引，返回这个id的工厂对应所有type类型的权值指针
    std::queue<int> busyRobotQueue_;
    std::queue<int> readyRobotQueue_;
    int nodeTotalNum_;
    int factoryIDShift_;

    const double INFINITE_ = 99999.0; // 表示无穷大
    const double MAX_ENCOURAGE_ = 0.0; // 表示最大激励
    const double ZERO_ = 0.1; // 表示0?
    const double distanceCoefficient_ = 10;
    const double chargeVelocity_ = 6.0; // 机器人预想冲刺速度，用于RFBroadcastUpdate中调整机器人-工厂是否忽略产品状态

};
#endif //HUAWEI_DISTRIBUTOR_H
