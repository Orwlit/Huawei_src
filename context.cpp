//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <vector>
#include <map>

#include "context.h"

Context::Context() {
    this->OK_ = "OK";
    robotTotalNum_ = 4;

    robots.resize(this->robotTotalNum_);


    if(!(this->Initialize())){
        std::cerr << "Initialization Failed" << std::endl;
    }


    this->SYSTEM_ENABLE_ = true;
}

bool Context::UpdateAllStatus(std::vector<Robot*> robots, std::map<FactoryType, std::vector<Factory*>> factories) {
    if (!(std::cin >> frameID_ >> currentMoney_)) {
        return false;
    }

    if (!(std::cin >> factoryTotalNum_)) {
        return false;
    }

    for (int i = 0; i < factoryTotalNum_; i++) {
        int factoryType_cache;
        int remainingTime_cache;
        int warehouseStatus_cache;
        int productStatus_cache;
        float x, y;

        //读取对应的工厂信息
        if (!(std::cin >> factoryType_cache >> x >> y
                >> remainingTime_cache >> warehouseStatus_cache >> productStatus_cache)) {
            return false;
        }
        //更新对应的工厂信息



    }

    int nearbyFactoryID_cahce; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_cahce; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_cahce; //角速度
    float linearVelocity_cahce; //线速度
    float orientation_cahce; //朝向
    std::vector<float> coordinate_cahce; //坐标


    for (int i = 0; i < robotTotalNum_; i++) {
        std::vector<float> robotData;
        for (int j = 0; j < 10; j++) {
            float value;
            if (!(std::cin >> value)) {
                return false;
            }
            robotData.push_back(value);
        }
        robots.push_back(robotData);
    }

    std::string ok_get;
    if (!(std::cin >> ok_get) || ok_get != "OK") {
        return false;
    }

    return true;
}

bool Context::Initialize(std::vector<Robot *> robots, std::map<FactoryType, std::vector<Factory *>> factories) {
    return false;
}
