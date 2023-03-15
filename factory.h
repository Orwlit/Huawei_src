//
// Created by orw on 3/14/23.
//
#pragma once
#include <vector>

#ifndef HUAWEI_FACTORY_H
#define HUAWEI_FACTORY_H

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
    FactoryType type_;
    FactoryFlag flag_;
    std::vector<float> coordinate_;
    int remainingProducingTime_; //剩余生产帧数: -1表示没有生产 0表示生产因输出格满而阻塞 >=0表示剩余生产帧数
    std::vector<int> warehouseStatus_; //仓库格状态
    int productState_; //产品格状态
public:
    Factory();
    bool setType(int type);

};

#endif //HUAWEI_FACTORY_H
