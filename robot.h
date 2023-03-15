//
// Created by orw on 3/14/23.
//
#pragma once
#include <vector>

#ifndef HUAWEI_ROBOT_H
#define HUAWEI_ROBOT_H

enum RobotFlag{
    Ready = 0,
    BUSY = 1
};

enum Priority{
     NORMAL = 0
};

class Robot{
private:

    void Forward(float speed);
    void Rotate(float speed);
    void Buy();
    void Sell();
    void Destroy();

    int robotID_;
    int nearbyFactoryID_; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_; //角速度
    float linearVelocity_; //线速度
    float orientation_; //朝向
    std::vector<float> coordinate_; //坐标
public:
    Robot();
    Robot(std::vector<float> coordinate_initial, float orientation_initial);
    void TrickLoop(); //偷鸡策略，围着19绕圈
    void HighSpeedMove(); //高速模式
    void LowSpeedMove();
};

#endif //HUAWEI_ROBOT_H
