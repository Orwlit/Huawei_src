//
// Created by orw on 3/14/23.
//
#ifndef CODECRAFTSDK_CONTEXT_H
#define CODECRAFTSDK_CONTEXT_H

#pragma once
#include <map>
#include <vector>
#include <memory>
#include <iostream>
#include <map>
#include <cmath>

#include "robot.h"
#include "factory.h"

class Context{
public:
    Context();
    bool Initialize();

    bool UpdateAllStatus();
    bool GenerateHistoryGraph();
    void FactoriesClassification();

    // 基本工具
    std::map<FactoryType, std::pair<bool, bool>> WarehouseStateConversion(FactoryType type, int rawInfo);
    void PrintHistoryMap(const std::vector<std::vector<double>> &map, const std::string &title);
    static void PrintMapMapVector(const std::map<FactoryType, std::map<FactoryType, std::vector<int>>> &map, const std::string &title) ;

    // 各种对象间的距离
    [[nodiscard]] double DistanceFF(int factory1_ID, int factory2_ID) const;
    [[nodiscard]] double DistanceFR(int robotID, int factoryID) const;
    [[nodiscard]] double DistanceRR(int robot1_ID, int robot2_ID) const;

    // 防撞策略
    [[nodiscard]] bool AboutToCrash(const std::shared_ptr<Robot> &robot1, const std::shared_ptr<Robot> &robot2, float k) const;

    //Getter
    [[nodiscard]] float GetDt() const;
    [[nodiscard]] int GetFrameId() const;
    [[nodiscard]] int GetPreviousFrameId() const;
    [[nodiscard]] int GetMAX_Hz() const;
    [[nodiscard]] int GetCurrentMoney() const;
    [[nodiscard]] bool IsSystemEnable() const;
    [[nodiscard]] const std::vector<std::shared_ptr<Robot>> &GetAllRobots() const;
    [[nodiscard]] const std::vector<std::shared_ptr<Factory>> &GetAllFactories() const;
    [[nodiscard]] std::shared_ptr<Robot> GetRobot(int robotIndex) const;
    [[nodiscard]] std::shared_ptr<Factory> GetFactory(int factoryIndex) const;
    [[nodiscard]] const std::map<FactoryType, std::map<FactoryType, std::vector<int>>> &GetGlobalFactoryTypeMap() const;
    [[nodiscard]] const std::vector<std::vector<double>> &GetInitialHistoryGraph() const;
    [[nodiscard]] const int GetNodeTotalNum() const;
    [[nodiscard]] const int GetFactoryTotalNum() const;
    [[nodiscard]] const int GetRobotTotalNum() const;

    //Setter
    void SetPreviousFrameId(int previousFrameId);

    std::vector<int> sellersNode_;

private:
    std::vector<std::shared_ptr<Robot>> allRobots_;
    std::vector<std::shared_ptr<Factory>> allFactories_;
    std::vector<std::shared_ptr<Factory>> oneFactories_;
    std::vector<std::shared_ptr<Factory>> twoFactories_;
    std::vector<std::shared_ptr<Factory>> threeFactories_;
    std::vector<std::shared_ptr<Factory>> fourFactories_;
    std::vector<std::shared_ptr<Factory>> fiveFactories_;
    std::vector<std::shared_ptr<Factory>> sixFactories_;
    std::vector<std::shared_ptr<Factory>> sevenFactories_;
    std::vector<std::shared_ptr<Factory>> eightFactories_;
    std::vector<std::shared_ptr<Factory>> nineFactories_;
    std::vector<int> oneFactoriesIndex_;
    std::vector<int> twoFactoriesIndex_;
    std::vector<int> threeFactoriesIndex_;
    std::vector<int> fourFactoriesIndex_;
    std::vector<int> fiveFactoriesIndex_;
    std::vector<int> sixFactoriesIndex_;
    std::vector<int> sevenFactoriesIndex_;
    std::vector<int> eightFactoriesIndex_;
    std::vector<int> nineFactoriesIndex_;

    std::map<FactoryType, std::map<FactoryType, std::vector<int>>> globalFactoryTypeMap_; //按工厂类型的索引，例如FACTORY_4到MATERIAL_1的全部工厂ID
    std::vector<int> routeRobotSeller_; //机器人到卖家的最短距离，第一个值为机器人ID，后面是路线上的工厂ID

    std::vector<std::vector<double>> initialHistoryGraph_;

    int frameID_;
    int previousFrameID_;
    int currentMoney_;
    int factoryTotalNum_;
    int nodeTotalNum_;
    float dt;
    bool SYSTEM_ENABLE_;

    const int MAX_HZ = 50;
    const int robotTotalNum_ = 4;
    const int factoryIDShift_ = robotTotalNum_; // 因为前四个为机器人，所以所有工厂序号做相应的偏移
    const std::string OK_ = "OK";
    const double MINIMUM_EQUAL_VALUE_ = 0.001;
    const double INFINITE_ = 99999.0; // 表示无穷大

};

#endif //CODECRAFTSDK_CONTEXT_H




