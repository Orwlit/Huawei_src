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
    bool run();

private:
    std::vector<std::shared_ptr<Robot>> robots_;
    std::map<int, std::shared_ptr<Factory>> allFactories_;

//    std::map<int, float*> factory_coordinate_;


    int frameID_;
    int currentMoney_;
    int factoryTotalNum_;
    int robotTotalNum_;

    std::string OK_;
    bool SYSTEM_ENABLE_;
    float MINIMUM_EQUAL_VALUE;
};

#endif //CODECRAFTSDK_CONTEXT_H
