//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <cmath>

#include "robot.h"

Robot::Robot() {

}

Robot::Robot(int robotID, float x_initial, float y_initial) {
    this->robotID_ = robotID;
    this->flag_ = RobotFlag::ROBOT_READY;
//    this->priority = Priority::ANYWAY;
    this->coordinate_[0] = x_initial;
    this->coordinate_[1] = y_initial;

    this->MAX_LINEAR_VELOCITY_ = 6.0f;
    this->MIN_LINEAR_VELOCITY_ = -2.0f;
    this->MAX_ANGULAR_VELOCITY_ = M_PI;
    this->MIN_ANGULAR_VELOCITY_ = -M_PI;
}

void Robot::TrickLoop() {

}

void Robot::HighSpeedMove(float destination_x, float destination_y, float dt) {
    // 14帧会让6减速到0
    float SLOW_DOWN_DISTANCE = 14.0f * dt * this->MAX_LINEAR_VELOCITY_;

    float distance_x = destination_x - this->GetCoordinate()[0];
    float distance_y = destination_y - this->GetCoordinate()[1];
    float distance = sqrt(pow(distance_x, 2) +pow(distance_y, 2));

    float theta = this->GetOrientation() - std::atan2(distance_y, distance_x);
    if (std::abs(theta) > M_PI){
        if (theta > 0){
            theta -= 2 * M_PI;
        }else {
            theta += 2 * M_PI;
        }
    }

    float k = 2.0f; // 可调系数
    if (theta > M_PI / 2){
        if (distance < SLOW_DOWN_DISTANCE){
            this->Forward(1);
        }
        this->RotateAngular(theta, dt);
    }else {
        if (distance > SLOW_DOWN_DISTANCE){
            this->Forward(6);
        }
        this->RotateAngular(theta, dt);
    }


//    float current_distance_x = destination_x - this->coordinate_[0];
//    float current_distance_y = destination_y - this->coordinate_[1];
////    std::cerr << "distance_x: " << current_distance_x << ", distance_y: " << current_distance_y << std::endl;
//    double distance = sqrt(pow(current_distance_x, 2) + pow(current_distance_y, 2));
//
//    //需要考虑atan关于原点对称的性质，应该向3象限而被误认为1象限
//    double current_theta = 0;
//    if (current_distance_x <= 0){ //第2,3象限
//        current_theta = M_PI + std::atan(current_distance_y / current_distance_x);
//    }
//    else{
//        if (current_distance_y >= 0){
//            current_theta = std::atan(current_distance_y / current_distance_x);
//        }
//        else {
//            current_theta = 2 * M_PI + std::atan(current_distance_y / current_distance_x);
//        }
//    }
//
//    auto d_theta = static_cast<float>(this->orientation_ - current_theta);
//    if (d_theta > M_PI){
//        d_theta = -1.0f * d_theta;
//
//    }
//
//    this->RotateAngular(d_theta, dt);

//    double max_rotation_in_dt = this->MAX_ANGULAR_VELOCITY_ * dt;
//
//    double condition = std::abs(d_theta) - std::abs(max_rotation_in_dt);
////    std::cerr << "Robot NO." << this->robotID_
////              << " pose: (" << this->coordinate_[0] << ", " << this->coordinate_[1] << ")"
////              << " orientation_: " << (180 * this->orientation_ / M_PI) << ", current_theta: " << (180 * current_theta / M_PI) << ", d_theta: " << (180 * d_theta / M_PI)
////              << " max_rotation: " << (180 * max_rotation_in_dt / M_PI) << std::endl;
//
//    if (condition > 0){ //dt时间内最大角速度不足以补齐d_theta
//        if (d_theta > 0){
//            this->Rotate(this->MIN_ANGULAR_VELOCITY_);
////            std::cerr << "MAX_ROTATE+, " << "Rotate rate" << this->MIN_ANGULAR_VELOCITY_ << std::endl;
//        }
//        if (d_theta < 0){
//            this->Rotate(this->MAX_ANGULAR_VELOCITY_);
////            std::cerr << "MAX_ROTATE-" << std::endl;
//        }
//    }
//    else{
//        this->Rotate(-d_theta / dt);
////        std::cerr << "low speed d_theta * dt: " << d_theta * dt << std::endl;
//    }


//    this->Forward(5.0);
}

void Robot::LowSpeedMove(float destination_x, float destination_y, float dt) {


}



void Robot::Forward(float speed) const {
    std::cout << "forward" << " " << robotID_ << " " << speed << std::endl;
}

void Robot::Rotate(float speed) const {




    std::cout << "rotate" << " " << robotID_ << " " << speed << std::endl;
}

bool Robot::Buy(int factoryID) const{
    if (factoryID < 0){
        std::cerr << "Robot NO." << this->robotID_
                  << " BUY input factoryID NOT accepted, input factoryID: " << factoryID << std::endl;
        return false;
    }
    if (factoryID != this->nearbyFactoryID_){
//        std::cerr << "Robot NO." << this->robotID_
//                  << " NOT able to BUY from " << factoryID << ", because NOT around. "
//                  << "nearby FactoryID is: " << this->nearbyFactoryID_ << std::endl;
        return false;
    }
    std::cout << "buy" << " " << robotID_ << std::endl;
    return true;
};

bool Robot::Sell(int factoryID) const{
    if (factoryID < 0){
        std::cerr << "Robot NO." << this->robotID_
                  << " SELL input factoryID NOT accepted, input factoryID: " << factoryID << std::endl;
        return false;
    }
    if (factoryID != this->nearbyFactoryID_){
//        std::cerr << "Robot NO." << this->robotID_
//                  << " NOT able to SELL from " << factoryID << ", because NOT around. "
//                  << "nearby FactoryID is: " << this->nearbyFactoryID_ << std::endl;
        return false;
    }
    std::cout << "sell" << " " << robotID_ << std::endl;

    return true;
};

bool Robot::Destroy() const{
    std::cout << "destroy" << " " << robotID_ << std::endl;
    return true;
};

bool Robot::SetNearbyFactoryID(int nearbyFactory) {
    this->nearbyFactoryID_ = nearbyFactory;
    return true;
}

bool Robot::SetCarryingType(int carryingType) {
    this->carryingType_ = carryingType;
    //
//    if (carryingType == 0){
//        this->flag_ = ROBOT_READY;
//    }
//    else{
//        this->flag_ = ROBOT_BUSY;
//    }
    return true;
}

bool Robot::SetPunishments(float timePunishment, float crashPunishment) {
    this->timePunishment_ = timePunishment;
    this->crashPunishment_ = crashPunishment;
    return true;
}

bool Robot::SetAngularVelocity(float angularVelocity) {
    this->angularVelocity_ = angularVelocity;
    return true;
}

bool Robot::SetLinearVelocity(float linearVelocity_x, float linearVelocity_y) {
//    if (linearVelocity_x > this->MAX_LINEAR_VELOCITY_ || linearVelocity_x < this->MIN_LINEAR_VELOCITY_){
//        std::cerr << "Robot " << this->robotID_ << " LinearVelocity_x OUT OF RANGE!!!" << std::endl;
//        return false;
//    }
//    if (linearVelocity_y > this->MAX_LINEAR_VELOCITY_ || linearVelocity_y < this->MIN_LINEAR_VELOCITY_){
//        std::cerr << "Robot " << this->robotID_ << " LinearVelocity_y OUT OF RANGE!!! "
//                  << "linearVelocity_y: " << linearVelocity_y << std::endl;
//
//        return false;
//    }
    double linear_velocity = sqrt(pow(linearVelocity_x, 2) + pow(linearVelocity_y, 2));
    double velocity_theta = atan2(linearVelocity_y, linearVelocity_x);
//    if ((velocity_theta * this->orientation_) < 0){
//        this->linearVelocity_ = -1.0f * static_cast<float>(linear_velocity);
//    }else{
    this->linearVelocity_ = static_cast<float>(linear_velocity); //TODO: 没考虑倒车
//    }
    this->linearVelocity_x_ = linearVelocity_x;
    this->linearVelocity_y_ = linearVelocity_y;
//    std::cerr << "车速度：" << this->linearVelocity_ << std::endl;
    return true;
}

bool Robot::SetOrientation(float orientation) {
    if (orientation >= M_PI || orientation <= -M_PI){
        std::cerr << "Robot Orientation OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
//        // 将朝向均变为正的
//        if (orientation > 0){
//            this->orientation_ = orientation;
//        } else{
//            this->orientation_ = static_cast<float>(2.0 * M_PI + orientation);
//        }
        this->orientation_ = orientation;
        return true;
    }
}

bool Robot::SetCoordinate(float x, float y) {
    if (x > 55.0 || y > 55.0){
        std::cerr << "Robot Coordinate OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
        this->coordinate_[0] = x;
        this->coordinate_[1] = y;
        return true;
    }
}

int Robot::GetRobotID() const {
    return this->robotID_;
}

bool Robot::Reachable(float destination_x, float destination_y) const {
    float current_distance_x = destination_x - this->coordinate_[0];
    float current_distance_y = destination_y - this->coordinate_[1];
    double distance = sqrt(pow(current_distance_x, 2) + pow(current_distance_y, 2));

    if ((this->flag_ == ROBOT_READY) && (distance < this->radiusBasic_)){
        return true;
    }
    else if ((this->flag_ == ROBOT_BUSY) && (distance < this->radiusCarry_)){
        return true;
    }

    return false;
}

RobotFlag Robot::GetFlag() const {
    return flag_;
}

int Robot::GetNearbyFactoryId() const {
    return nearbyFactoryID_;
}

int Robot::GetCarryingType() const {
    return carryingType_;
}

float Robot::GetAngularVelocity() const {
    return angularVelocity_;
}

float Robot::GetLinearVelocity() const {
    return linearVelocity_;
}

float Robot::GetOrientation() const {
    return orientation_;
}

const float *Robot::GetCoordinate() const {
    return coordinate_;
}

void Robot::RotateAngular(float angular, float dt) const {
    double max_rotation_in_dt = this->MAX_ANGULAR_VELOCITY_ * dt;
    double condition = std::abs(angular) - std::abs(max_rotation_in_dt);

    if (condition > 0){ //dt时间内最大角速度不足以补齐d_theta
        if (angular > 0){
            this->Rotate(this->MIN_ANGULAR_VELOCITY_);
//            std::cerr << "MAX_ROTATE+, " << "Rotate rate" << this->MIN_ANGULAR_VELOCITY_ << std::endl;
        }
        if (angular < 0){
            this->Rotate(this->MAX_ANGULAR_VELOCITY_);
//            std::cerr << "MAX_ROTATE-" << std::endl;
        }
    }
    else{
        this->Rotate(-angular / dt);
//        std::cerr << "low speed d_theta * dt: " << d_theta * dt << std::endl;
    }
}

float Robot::GetLinearVelocityX() const {
    return linearVelocity_x_;
}

float Robot::GetLinearVelocityY() const {
    return linearVelocity_y_;
}

void Robot::SetFlag(RobotFlag flag) {
    this->flag_ = flag;
}


