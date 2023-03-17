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
    this->flag = ROBOT_READY;
    this->coordinate_[0] = x_initial;
    this->coordinate_[1] = y_initial;



    this->MAX_LINEAR_VELOCITY = 6.0f;
    this->MIN_LINEAR_VELOCITY = -2.0f;
    this->MAX_ANGULAR_VELOCITY = M_PI;
    this->MIN_ANGULAR_VELOCITY = -M_PI;
}

void Robot::TrickLoop() {

}

void Robot::HighSpeedMove(float destination_x, float destination_y, float dt) {
    float current_distance_x = destination_x - this->coordinate_[0];
    float current_distance_y = destination_y - this->coordinate_[1];
//    std::cerr << "distance_x: " << current_distance_x << ", distance_y: " << current_distance_y << std::endl;
    double distance = sqrt(pow(current_distance_x, 2) + pow(current_distance_y, 2));

    //需要考虑atan关于原点对称的性质，应该向3象限而被误认为1象限
    double current_theta = 0;
    if (current_distance_x <= 0){ //第2,3象限
        current_theta = M_PI + std::atan(current_distance_y / current_distance_x);
    }
    else{
        if (current_distance_y >= 0){
            current_theta = std::atan(current_distance_y / current_distance_x);
        }
        else {
            current_theta = 2 * M_PI + std::atan(current_distance_y / current_distance_x);
        }
    }

    auto d_theta = static_cast<float>(this->orientation_ - current_theta);
    if (d_theta > M_PI){
        d_theta = -1.0f * d_theta;
    }
    double max_rotation_in_dt = this->MAX_ANGULAR_VELOCITY * dt;

    double condition = std::abs(d_theta) - std::abs(max_rotation_in_dt);
//    std::cerr << "Robot NO." << this->robotID_
//              << " pose: (" << this->coordinate_[0] << ", " << this->coordinate_[1] << ")"
//              << " orientation_: " << (180 * this->orientation_ / M_PI) << ", current_theta: " << (180 * current_theta / M_PI) << ", d_theta: " << (180 * d_theta / M_PI)
//              << " max_rotation: " << (180 * max_rotation_in_dt / M_PI) << std::endl;

    if (condition > 0){ //dt时间内最大角速度不足以补齐d_theta
        if (d_theta > 0){
            this->Rotate(this->MIN_ANGULAR_VELOCITY);
//            std::cerr << "MAX_ROTATE+, " << "Rotate rate" << this->MIN_ANGULAR_VELOCITY << std::endl;
        }
        if (d_theta < 0){
            this->Rotate(this->MAX_ANGULAR_VELOCITY);
//            std::cerr << "MAX_ROTATE-" << std::endl;
        }
    }
    else{
        this->Rotate(-d_theta * dt);
//        std::cerr << "low speed d_theta * dt: " << d_theta * dt << std::endl;
    }


    this->Forward(6.0);
}

void Robot::LowSpeedMove() {

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
        std::cerr << "Robot NO." << this->robotID_
                  << " NOT able to BUY from " << factoryID << ", because NOT around. "
                  << "nearby FactoryID is: " << this->nearbyFactoryID_ << std::endl;
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
        std::cerr << "Robot NO." << this->robotID_
                  << " NOT able to SELL from " << factoryID << ", because NOT around. "
                  << "nearby FactoryID is: " << this->nearbyFactoryID_ << std::endl;
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
    if (carryingType == 0){
        this->flag = ROBOT_READY;
    }
    else{
        this->flag = ROBOT_BUSY;
    }
    return true;
}

bool Robot::SetPunishments(float timePunishment, float crashPunishment) {
    this->timePunishment_ = timePunishment;
    this->crashPunishment_ = crashPunishment;
    return true;
}

bool Robot::SetAngularVelocity(float angularVelocity) {
    if (angularVelocity > this->MAX_ANGULAR_VELOCITY || angularVelocity < this->MIN_ANGULAR_VELOCITY){
        std::cerr << "Robot AngularVelocity OUT OF RANGE!!!" << std::endl;
        std::cerr << "AngularVelocity: " << angularVelocity
                  << "should in (" << this->MAX_ANGULAR_VELOCITY << ", " << this->MIN_ANGULAR_VELOCITY << ")" << std::endl;
        return false;
    }
    else{
        this->angularVelocity_ = angularVelocity;
        return true;
    }
}

bool Robot::SetLinearVelocity(float linearVelocity_x, float linearVelocity_y) {
//    if (linearVelocity_x > this->MAX_LINEAR_VELOCITY || linearVelocity_x < this->MIN_LINEAR_VELOCITY){
//        std::cerr << "Robot " << this->robotID_ << " LinearVelocity_x OUT OF RANGE!!!" << std::endl;
//        return false;
//    }
//    if (linearVelocity_y > this->MAX_LINEAR_VELOCITY || linearVelocity_y < this->MIN_LINEAR_VELOCITY){
//        std::cerr << "Robot " << this->robotID_ << " LinearVelocity_y OUT OF RANGE!!! "
//                  << "linearVelocity_y: " << linearVelocity_y << std::endl;
//
//        return false;
//    }
    double linear_velocity = sqrt(pow(linearVelocity_x, 2) + pow(linearVelocity_y, 2));
    if (linear_velocity > this->MAX_LINEAR_VELOCITY){
        std::cerr << "Robot " << this->robotID_ << " linear_velocity OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
        this->linearVelocity_x_ = linearVelocity_x;
        this->linearVelocity_y_ = linearVelocity_y;
        this->linearVelocity_ = static_cast<float>(linear_velocity); //TODO: 没考虑倒车
        return true;
    }
}

bool Robot::SetOrientation(float orientation) {
    if (orientation >= M_PI || orientation <= -M_PI){
        std::cerr << "Robot Orientation OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
        if (orientation > 0){
            this->orientation_ = orientation;
        } else{
            this->orientation_ = static_cast<float>(2.0 * M_PI + orientation);
        }
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

    if ((this->flag == ROBOT_READY) && (distance < this->radiusBasic)){
        return true;
    }
    else if ((this->flag == ROBOT_BUSY) && (distance < this->radiusCarry)){
        return true;
    }

    return false;
}


