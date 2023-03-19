//
// Created by orw on 3/14/23.
//
#ifndef HUAWEI_ROBOT_H
#define HUAWEI_ROBOT_H

#include <vector>

//#include "distributor.h"

enum RobotFlag{
    ROBOT_READY = 0,
    ROBOT_BUSY = 1
};

class Robot{
private:


    int robotID_;
    RobotFlag flag;
//    Priority priority;

    float MAX_LINEAR_VELOCITY;
    float MAX_ANGULAR_VELOCITY;
    float MIN_LINEAR_VELOCITY;
    float MIN_ANGULAR_VELOCITY;

    float radiusBasic, radiusCarry;

    int nearbyFactoryID_; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_; //角速度
    float linearVelocity_, linearVelocity_x_, linearVelocity_y_; //线速度 x, y
    float orientation_; //朝向
    float coordinate_[2]; //坐标


public:
    Robot();
    Robot(int robotID, float x_initial, float y_initial);

    void Forward(float speed) const;
    void Rotate(float speed) const;
    void RotateAngular(float angular, float dt) const;

    void TrickLoop(); //偷鸡策略，围着19绕圈
    void HighSpeedMove(float destination_x, float destination_y, float d_time); //高速模式
    void LowSpeedMove(float destination_x, float destination_y, float dt);

    //Setter
    bool SetNearbyFactoryID(int nearbyFactory);
    bool SetCarryingType(int carryingType);
    bool SetPunishments(float timePunishment, float crashPunishment);
    bool SetAngularVelocity(float angularVelocity);
    bool SetLinearVelocity(float linearVelocity_x, float linearVelocity_y);
    bool SetOrientation(float orientation);
    bool SetCoordinate(float x, float y);

    //Getter
    [[nodiscard]] int GetRobotID() const ;
    [[nodiscard]] RobotFlag GetFlag() const;
    [[nodiscard]] int GetNearbyFactoryId() const;
    [[nodiscard]] int GetCarryingType() const;
    [[nodiscard]] float GetAngularVelocity() const;
    [[nodiscard]] float GetLinearVelocity() const;
    [[nodiscard]] float GetOrientation() const;
    [[nodiscard]] const float *GetCoordinate() const;

    //Interact
    [[nodiscard]] bool Buy(int factoryID) const;
    [[nodiscard]] bool Sell(int factoryID) const;
    [[nodiscard]] bool Destroy() const;
    [[nodiscard]] bool Reachable(float destination_x, float destination_y) const;

};

#endif //HUAWEI_ROBOT_H
