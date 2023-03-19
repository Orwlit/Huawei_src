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
    bool UpdateGraph();

    //Distance between Factories or Robots
    [[nodiscard]] float DistanceFF(int factory1_ID, int factory2_ID) const;
    [[nodiscard]] float DistanceFR(int robotID, int factoryID) const;
    [[nodiscard]] float DistanceRR(int robot1_ID, int robot2_ID) const;

    //防撞策略
    bool AboutToCrash(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2, float k) const;

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

    float **globalGraph_;

//    std::map<int, float*> factory_coordinate_;


    int frameID_;
    int previousFrameID_;
    int MAX_HZ;
    int currentMoney_;
    int factoryTotalNum_;
    int robotTotalNum_;

    float dt;
    std::string OK_;
    bool SYSTEM_ENABLE_;
    float MINIMUM_EQUAL_VALUE_;
};

#endif //CODECRAFTSDK_CONTEXT_H




