//
// Created by orw on 3/14/23.
//
#ifndef HUAWEI_FACTORY_H
#define HUAWEI_FACTORY_H

#pragma once
#include <vector>

enum FactoryFlag{
    Ready = 0,
    Producing = 1,
    Vacant = 2
};

enum FactoryType{
    UNKNOWN = 0, //一开始均设为UNKNOWN
    MATERIAL_1 = 1,
    MATERIAL_2 = 2,
    MATERIAL_3 = 3,
    FACTORY_4 = 4,
    FACTORY_5 = 5,
    FACTORY_6 = 6,
    FACTORY_7 = 7,
    SELLER_8 = 8,
    SELLER_9 = 9
};

class Factory{
private:
    FactoryType factoryType_;
    FactoryFlag factoryFlag_;
    float coordinate_[2];
    int remainingTime_; //剩余生产帧数: -1表示没有生产 0表示生产因输出格满而阻塞 >=0表示剩余生产帧数
    int warehouseState_; //仓库格状态
    int productState_; //产品格状态
public:
    Factory();
    Factory(FactoryType factoryType, float x_initial, float y_initial);;
    float* GetCoordinate();
    bool SetType(FactoryType type);
    bool SetRemainingTime(int time);
    bool SetWarehouseStatus(int state);
    bool SetProductStatus(int state);


};

#endif //HUAWEI_FACTORY_H
