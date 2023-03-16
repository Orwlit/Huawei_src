//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <vector>
#include <map>
#include <cmath>

#include "context.h"

Context::Context() {
    this->OK_ = "OK";
    robotTotalNum_ = 4;
    MINIMUM_EQUAL_VALUE = 0.01;

    if(!(this->Initialize())){
        std::cerr << "Initialization Failed" << std::endl;
    }
    std::cout << "OK" << std::endl;

    this->SYSTEM_ENABLE_ = true;
}

bool Context::UpdateAllStatus(std::vector<Robot*> robots,
                              std::map<FactoryType, std::vector<Factory*>> factories) {
    if (!(std::cin >> this->frameID_ >> this->currentMoney_)) {
        std::cerr << "UPDATE FRAME_ID or CURRENT_MONEY FAILED!!!" << std::endl;
        return false;
    }

    if (!(std::cin >> this->factoryTotalNum_)) {
        std::cerr << "UPDATE FactoryTotalNum FAILED!!!" << std::endl;
        return false;
    }

    int factoryType_cache;
    int remainingTime_cache;
    int warehouseStatus_cache;
    int productStatus_cache;
    float factory_x_input, factory_y_input;
    for (int i = 0; i < this->factoryTotalNum_; i++) {
        //读取对应的工厂信息
        if (!(std::cin >> factoryType_cache >> factory_x_input >> factory_y_input
                       >> remainingTime_cache >> warehouseStatus_cache >> productStatus_cache)) {
            std::cerr << "FACTORY INFO READ FAILED!!!" << std::endl;
            return false;
        }

        //更新对应的工厂信息
        auto it = factories.find(static_cast<FactoryType>(factoryType_cache));
        //根据坐标查找对应工厂
        for (Factory* factory : it->second) {
            double distance = 0.0;
            float factory_x = factory->GetCoordinate()[0];
            float factory_y = factory->GetCoordinate()[1];

            distance = sqrt(pow((factory_x - factory_x_input), 2) + pow((factory_y - factory_y_input), 2));
            if (distance < this->MINIMUM_EQUAL_VALUE){
                //刷新工厂类型，希望不刷新。若刷新了，则打印错误信息到cerr
                if (factory->SetType(static_cast<FactoryType>(factoryType_cache))){
                    std::cerr << "Some Factory has CHANGED TYPE!!!" << std::endl;
                }
                //刷新其他信息
                if (!(factory->SetRemainingTime(remainingTime_cache))){
                    std::cerr << "Some Factory SetRemainingTime FAILED!!!" << std::endl;
                }
                if (!(factory->SetWarehouseStatus(warehouseStatus_cache))){
                    std::cerr << "Some Factory SetWarehouseStatus FAILED!!!" << std::endl;
                }
                if (!(factory->SetProductStatus(productStatus_cache))){
                    std::cerr << "Some Factory SetProductStatus FAILED!!!" << std::endl;
                }
                break; //找到对应的工厂，且状态更新结束
            }
        }



    }

    int nearbyFactoryID_cahce; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_cahce; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_cahce; //角速度
    float linearVelocity_cahce; //线速度
    float orientation_cahce; //朝向
    float robot_x_input, robot_y_input; //坐标

    //读取对应的机器人信息
    if (!(std::cin >> nearbyFactoryID_cahce >> carryingType_cahce
                   >> timePunishment_cahce >> crashPunishment_cahce
                   >> angularVelocity_cahce >> linearVelocity_cahce
                   >> orientation_cahce >> robot_x_input >> robot_y_input)) {
        std::cerr << "ROBOT INFO READ FAILED!!!" << std::endl;
        return false;
    }

    //更新对应的机器人信息
    for (auto robot : robots) {
        if (!(robot->SetNearbyFactoryID(nearbyFactoryID_cahce))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetNearbyFactoryID FAILED!!!" << std::endl;
        }
        if (!(robot->SetCarryingType(carryingType_cahce))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetCarryingType FAILED!!!" << std::endl;
        }
        if (!(robot->SetPunishments(timePunishment_cahce, crashPunishment_cahce))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetPunishments FAILED!!!" << std::endl;
        }
        if (!(robot->SetAngularVelocity(angularVelocity_cahce))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetAngularVelocity FAILED!!!" << std::endl;
        }
        if (!(robot->SetLinearVelocity(linearVelocity_cahce))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetLinearVelocity FAILED!!!" << std::endl;
        }
        if (!(robot->SetOrientation(orientation_cahce))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetOrientation FAILED!!!" << std::endl;
        }
        if (!(robot->SetCoordinate(robot_x_input, robot_y_input))){
            std::cerr << "ROBOT NO." << robot->GetRobotID() << " SetCoordinate FAILED!!!" << std::endl;
        }
    }

    //读取OK，判断信息结束
    std::string ok_get;
    if (!(std::cin >> ok_get) || ok_get != this->OK_) {
        return false;
    }
    return true;
}

bool Context::Initialize() {
    std::vector<std::string> initial_map;
    initial_map.resize(100);
    char c_cache;
    int robotID = 0;
    for (int col = 0; col < 100; ++col) {
//        std::getline(std::cin, initial_map[col]);
        for (int row = 0; row < 100; ++row) {
            std::cin >> c_cache;
            if (c_cache == '.'){
                continue;
            }

            float x = 0.25 + 0.5 * row;
            float y = 100.0 - 0.25 - 0.5 * col;
            if (c_cache == 'A'){
                robots.push_back(new Robot(robotID, x, y)); //danger new
                continue;
            }
            else{ //还剩数字1-9的工厂
                auto factoryType = static_cast<FactoryType>(c_cache);
                auto it = factories.find(factoryType);
                it->second.push_back(new Factory(factoryType, x, y));
                continue;
            }
        }
    }

    //读取OK，判断信息结束
    std::string ok_get;
    if (!(std::cin >> ok_get) || ok_get != this->OK_) {
        std::cerr << "Context INITIALIZATION NOT RECEIVE \"OK\"!!!" << std::endl;
        return false;
    }
    return true;
}

bool Context::run() {
    robots[0]->HighSpeedMove();
    return true;
}
