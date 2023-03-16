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
    this->coordinate_[0] = x_initial;
    this->coordinate_[1] = y_initial;



    this->MAX_LINEAR_VELOCITY = 6.0;
    this->MIN_LINEAR_VELOCITY = -2.0;
    this->MAX_ANGULAR_VELOCITY = M_PI;
    this->MIN_ANGULAR_VELOCITY = -M_PI;
}

void Robot::Forward(float speed) const {
    std::cout << "forward" << " " << robotID_ << " " << speed << std::endl;
}

void Robot::Rotate(float speed) const {
    std::cout << "rotate" << " " << robotID_ << " " << speed << std::endl;
}

void Robot::Buy() const{
    std::cout << "buy" << " " << robotID_ << std::endl;
};

void Robot::Sell() const{
    std::cout << "sell" << " " << robotID_ << std::endl;
};

void Robot::Destroy() const{
    std::cout << "destroy" << " " << robotID_ << std::endl;
};


void Robot::TrickLoop() {

}

void Robot::HighSpeedMove() {
    this->Forward(6.0);
}

void Robot::LowSpeedMove() {

}

bool Robot::SetNearbyFactoryID(int nearbyFactory) {
    this->nearbyFactoryID_ = nearbyFactory;
    return true;
}

bool Robot::SetCarryingType(int carryingType) {
    this->carryingType_ = carryingType;
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
        return false;
    }
    else{
        this->angularVelocity_ = angularVelocity;
        return true;
    }
}

bool Robot::SetLinearVelocity(float linearVelocity) {
    if (linearVelocity > this->MAX_LINEAR_VELOCITY || linearVelocity < this->MIN_LINEAR_VELOCITY){
        std::cerr << "Robot LinearVelocity OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
        this->linearVelocity_ = linearVelocity;
        return true;
    }
}

bool Robot::SetOrientation(float orientation) {
    if (orientation >= M_PI || orientation <= -M_PI){
        std::cerr << "Robot Orientation OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
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

int Robot::GetRobotID() {
    return this->robotID_;
}


