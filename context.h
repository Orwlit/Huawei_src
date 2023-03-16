//
// Created by orw on 3/14/23.
//
#ifndef CODECRAFTSDK_CONTEXT_H
#define CODECRAFTSDK_CONTEXT_H

#pragma once
#include <map>
#include <vector>
#include <mutex>

#include "robot.h"
#include "factory.h"

//用作更新机器人和工厂的信息，单例模式
class Subscriber{
private:


public:




};


class Context{
public:
    Context();
    bool Initialize();
    bool UpdateAllStatus(std::vector<Robot*> robots,
                         std::map<FactoryType, std::vector<Factory*>> factories);
    bool run();

private:
    std::vector<Robot*> robots;
    std::map<FactoryType, std::vector<Factory*>> factories;

    std::map<int, std::pair<int, int>> factory_coordinate;

    static std::mutex mtx; // 互斥锁

    int frameID_;
    int currentMoney_;
    int factoryTotalNum_;
    int robotTotalNum_;

    std::vector<int> producing_time; //?


    std::string OK_;
    bool SYSTEM_ENABLE_;
    float MINIMUM_EQUAL_VALUE;
};

#endif //CODECRAFTSDK_CONTEXT_H
