//
// Created by orw on 3/14/23.
//
#pragma once
#include <vector>

#ifndef HUAWEI_ROBOT_H
#define HUAWEI_ROBOT_H

//enum RobotFlag{
//    Ready = 0,
//    BUSY = 1
//};

enum Priority{
     NORMAL = 0
};

class Robot{
private:

    void Forward(float speed) const;
    void Rotate(float speed) const;


    int robotID_;
    float MAX_LINEAR_VELOCITY;
    float MAX_ANGULAR_VELOCITY;
    float MIN_LINEAR_VELOCITY;
    float MIN_ANGULAR_VELOCITY;

    int nearbyFactoryID_; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_; //角速度
    float linearVelocity_x_, linearVelocity_y_; //线速度 x, y
    float orientation_; //朝向
    float coordinate_[2]; //坐标
public:
    Robot();
    Robot(int robotID, float x_initial, float y_initial);

    bool SetNearbyFactoryID(int nearbyFactory);
    bool SetCarryingType(int carryingType);
    bool SetPunishments(float timePunishment, float crashPunishment);
    bool SetAngularVelocity(float angularVelocity);
    bool SetLinearVelocity(float linearVelocity_x, float linearVelocity_y);
    bool SetOrientation(float orientation);
    bool SetCoordinate(float x, float y);

    int GetRobotID();

    void Buy() const;
    void Sell() const;
    void Destroy() const;
    void TrickLoop(); //偷鸡策略，围着19绕圈
    void HighSpeedMove(); //高速模式
    void LowSpeedMove();
};

#endif //HUAWEI_ROBOT_H
