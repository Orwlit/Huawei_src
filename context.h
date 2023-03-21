//
// Created by orw on 3/14/23.
//
#ifndef CODECRAFTSDK_CONTEXT_H
#define CODECRAFTSDK_CONTEXT_H

#pragma once
#include <map>
#include <vector>
#include <memory>

#include "robot.h"
#include "factory.h"

class Context{
public:
    Context();
    bool Initialize();

    bool UpdateAllStatus();
    bool GenerateHistoryGraph();
    void FactoriesClassification();


    //Distance between Factories or Robots
    [[nodiscard]] float DistanceFF(int factory1_ID, int factory2_ID) const;
    [[nodiscard]] float DistanceFR(int robotID, int factoryID) const;
    [[nodiscard]] float DistanceRR(int robot1_ID, int robot2_ID) const;

    //防撞策略
    [[nodiscard]] bool AboutToCrash(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2, float k) const;

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

    //Setter
    void SetPreviousFrameId(int previousFrameId);

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
//    std::vector<std::vector<std::vector<FactoryType>>> globalFactoryMap_;
    std::map<FactoryType, std::map<FactoryType, std::vector<int>>> globalFactoryMap_;


    std::vector<std::vector<double>> initialHistoryGraph_;

//    std::map<int, float*> factory_coordinate_;


    int frameID_;
    int previousFrameID_;
    int currentMoney_;
    int factoryTotalNum_;
    float dt;
    bool SYSTEM_ENABLE_;

    const int MAX_HZ = 50;
    const int robotTotalNum_ = 4;
    const int factoryIDShift_ = robotTotalNum_; // 因为前四个为机器人，所以所有工厂序号做相应的偏移
    const std::string OK_ = "OK";
    const double MINIMUM_EQUAL_VALUE_ = 0.001;
    const double INFINITE_ = 999.0; // 表示无穷大

};

#endif //CODECRAFTSDK_CONTEXT_H




