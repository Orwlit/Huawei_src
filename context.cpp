//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

#include "context.h"

Context::Context() {
    this->OK_ = "OK";
    this->robotTotalNum_ = 4;
    this->MINIMUM_EQUAL_VALUE = 0.001f;
    this->MAX_HZ = 50;

    this->frameID_ = 0;
    this->previousFrameID = 0;
    if(!(this->Initialize())){
        std::cerr << "Initialization Failed" << std::endl;
    }
    std::cerr << "Initialization complete" << std::endl;
    std::cout << "OK" << std::flush;

    this->SYSTEM_ENABLE_ = true;
}

bool Context::UpdateAllStatus() {
    if (!(std::cin >> this->frameID_ >> this->currentMoney_)) {
        std::cerr << "UPDATE FRAME_ID or CURRENT_MONEY FAILED!!!" << std::endl;
        std::cerr << "frameID_: " << this->frameID_ << "\t"
                  << "currentMoney_: " << this->currentMoney_ << std::endl;
        return false;
    }

    if (!(std::cin >> this->factoryTotalNum_)) {
        std::cerr << "UPDATE FactoryTotalNum FAILED!!!" << std::endl;
        std::cerr << "factoryTotalNum_: " << factoryTotalNum_ << std::endl;
        return false;
    }

    int factoryType_cache;
    int remainingFrame_cache;
    int warehouseStatus_cache;
    int productStatus_cache;
    float factory_x_input, factory_y_input;
    for (int i = 0; i < this->factoryTotalNum_; i++) {
//        std::cerr << "---------------Reading Factory Info---------------" << std::endl;
        //读取对应的工厂信息
        if (!(std::cin >> factoryType_cache)) {
            std::cerr << "FACTORY Type READ FAILED!!!" << std::endl;
            std::cerr << "factoryType: " << factoryType_cache << std::endl;
            return false;
        }
//        std::cerr << "factoryType: " << factoryType_cache << std::endl;
        if (!(std::cin >> factory_x_input >> factory_y_input)) {
            std::cerr << "FACTORY Pose READ FAILED!!!" << std::endl;
            std::cerr << "x: " << factory_x_input << "\ty: " << factory_y_input << std::endl;
            return false;
        }
//        std::cerr << "x: " << factory_x_input << "\ty: " << factory_y_input << std::endl;
        if (!(std::cin >> remainingFrame_cache)) {
            std::cerr << "FACTORY Remaining Time READ FAILED!!!" << std::endl;
            std::cerr << "remainingTime: " << remainingFrame_cache << std::endl;
            return false;
        }
//        std::cerr << "remainingTime: " << remainingFrame_cache << std::endl;
        if (!(std::cin >> warehouseStatus_cache >> productStatus_cache)) {
            std::cerr << "FACTORY Space Status READ FAILED!!!" << std::endl;
            std::cerr << "warehouseStatus: " << warehouseStatus_cache << std::endl;
            std::cerr << "productStatus: " << productStatus_cache << std::endl;
            return false;
        }
//        std::cerr << "warehouseStatus: " << warehouseStatus_cache << std::endl;
//        std::cerr << "productStatus: " << productStatus_cache << std::endl;


//        std::cerr << "--------------Updating Factory info---------------" << std::endl;
        //更新对应的工厂信息 根据坐标查找对应工厂
        bool isUpdate = false;
        for (int factory_index = 0; factory_index < this->factoryTotalNum_; ++factory_index) {
            double distance = 0.0;
            const float factory_x = this->allFactories_[factory_index]->GetCoordinate()[0];
            const float factory_y = this->allFactories_[factory_index]->GetCoordinate()[1];

            distance = sqrt(pow((factory_x - factory_x_input), 2) + pow((factory_y - factory_y_input), 2));
            if (distance < this->MINIMUM_EQUAL_VALUE){
                //刷新工厂类型，希望不刷新。若刷新了，则打印错误信息到cerr
                if (this->allFactories_[factory_index]->SetType(static_cast<FactoryType>(factoryType_cache))){
                    std::cerr << "FactoryID: " << factory_index << " has CHANGED TYPE!!!" << std::endl;
                }
                //刷新其他信息
                if (!(this->allFactories_[factory_index]->SetRemainingFrame(remainingFrame_cache))){
                    std::cerr << "FactoryID: " << factory_index << " SetRemainingFrame FAILED" << std::endl;
                }
                if (!(this->allFactories_[factory_index]->SetWarehouseStatus(warehouseStatus_cache))){
                    std::cerr << "FactoryID: " << factory_index << " SetWarehouseStatus FAILED!!!" << std::endl;
                }
                if (!(this->allFactories_[factory_index]->SetProductStatus(productStatus_cache))){
                    std::cerr << "FactoryID: " << factory_index << " SetProductStatus FAILED!!!" << std::endl;
                }
                isUpdate = true;
                break; //找到对应的工厂，且状态更新结束
            }
        }
        if (!isUpdate){
            std::cerr << "Factory Coordinate (" << factory_x_input << ", " << factory_y_input << ") NOT FOUND !!!" << std::endl;
        }
    }

    int nearbyFactoryID_cahce; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_cahce; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_cahce; //角速度
    float linearVelocity_x_cahce, linearVelocity_y_cahce; //线速度 x, y
    float orientation_cahce; //朝向
    float robot_x_input, robot_y_input; //坐标

    //读取对应的机器人信息
    for (int robot_index = 0; robot_index < this->robotTotalNum_; ++robot_index) {
        std::cin >> nearbyFactoryID_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT nearbyFactoryID READ FAILED!!!" << std::endl;
            std::cerr << "nearbyFactoryID: " << nearbyFactoryID_cahce << std::endl;
            return false;
        }

        std::cin >> carryingType_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT carryingType READ FAILED!!!" << std::endl;
            std::cerr << "carryingType: " << carryingType_cahce << std::endl;
            return false;
        }

        std::cin >> timePunishment_cahce >> crashPunishment_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT Punishment READ FAILED!!!" << std::endl;
            return false;
        }

        std::cin >> angularVelocity_cahce >> linearVelocity_x_cahce >> linearVelocity_y_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT Velocity READ FAILED!!!" << std::endl;
            return false;
        }

        std::cin >> orientation_cahce >> robot_x_input >> robot_y_input;
        if (std::cin.bad()) {
            std::cerr << "ROBOT Pose READ FAILED!!!" << std::endl;
            return false;
        }

        //更新对应的机器人信息
        if (!(this->robots_[robot_index]->SetNearbyFactoryID(nearbyFactoryID_cahce))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetNearbyFactoryID FAILED!!!\t";
        }
        if (!(this->robots_[robot_index]->SetCarryingType(carryingType_cahce))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetCarryingType FAILED!!!\t";
        }
        if (!(this->robots_[robot_index]->SetPunishments(timePunishment_cahce, crashPunishment_cahce))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetPunishments FAILED!!!\t";
        }
        if (!(this->robots_[robot_index]->SetAngularVelocity(angularVelocity_cahce))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetAngularVelocity FAILED!!!\t";
        }
        if (!(this->robots_[robot_index]->SetLinearVelocity(linearVelocity_x_cahce, linearVelocity_y_cahce))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetLinearVelocity FAILED!!!\t";
        }
        if (!(this->robots_[robot_index]->SetOrientation(orientation_cahce))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetOrientation FAILED!!!\t";
        }
        if (!(this->robots_[robot_index]->SetCoordinate(robot_x_input, robot_y_input))) {
            std::cerr << "ROBOT NO." << this->robots_[robot_index]->GetRobotID()
                      << " SetCoordinate FAILED!!!\t";
        }
    }

    //读取OK，判断信息结束
    std::string ok_get;
    if (!(std::cin >> ok_get) || ok_get != this->OK_) {
        std::cerr << "UpdateAllStatus NOT OK!!!" << std::endl;
        std::cerr << "ok_get: " << ok_get << "\tthis->OK_: " << this->OK_ << std::endl;

        return false;
    }

    return true;
}

bool Context::Initialize() {
    std::vector<std::string> initial_map;
    initial_map.resize(100);
    int robotID = 0;
    int factoryID = 0;
    char current_char;
    for (int col = 0; col < 100; ++col) {
        std::getline(std::cin, initial_map[col]);
        for (int row = 0; row < 100; ++row) {
            current_char = initial_map[col][row];
            if (current_char == '.'){
                continue;
            }
            //计算当前字符对应的坐标
            float x = 0.25f + 0.5f * static_cast<float>(row);
            float y = 50.0f - 0.25f - 0.5f * static_cast<float>(col);

            if (current_char == 'A'){
                robots_.push_back(std::make_shared<Robot>(robotID, x, y));
                ++robotID;
                continue;
            }
            if (isdigit(current_char)){
                int tmp = current_char - '0'; //直接static_cast会让current_char变为ASCII码，50
                auto factoryType = static_cast<FactoryType>(tmp);
                allFactories_[factoryID] = std::make_shared<Factory>(factoryID, factoryType, x, y);
//                std::cerr << "factoryID: " << factoryID << " Created, x: " << x << " y: " << y << std::endl;
                ++factoryID;
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
    while (Context::UpdateAllStatus()){

        float dt = (static_cast<float>(this->frameID_ - this->previousFrameID)) * (1.0f / static_cast<float>(this->MAX_HZ));
        if (dt == 0){
            std::cerr << "dt = 0!!!, frameID_: " << this->frameID_ << ", previousFrameID: " << this->previousFrameID << std::endl;
        }
        std::cout << this->frameID_ << std::endl;
        robots_[0]->HighSpeedMove(2.0f, 2.0f, dt);
        std::cout << "OK" << std::flush;

        this->previousFrameID = this->frameID_;

    }
    return false;
}
