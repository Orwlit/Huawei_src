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
    this->MAX_ANGULAR_VELOCITY = M_PI + 0.1f;
    this->MIN_ANGULAR_VELOCITY = -M_PI - 0.1f;
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
    if (linearVelocity_x > this->MAX_LINEAR_VELOCITY || linearVelocity_x < this->MIN_LINEAR_VELOCITY){
        std::cerr << "Robot " << this->robotID_ << " LinearVelocity_x OUT OF RANGE!!!" << std::endl;
        return false;
    }
    if (linearVelocity_y > this->MAX_LINEAR_VELOCITY || linearVelocity_y < this->MIN_LINEAR_VELOCITY){
        std::cerr << "Robot " << this->robotID_ << " LinearVelocity_y OUT OF RANGE!!!" << std::endl;
        return false;
    }
    double linear_velocity = sqrt(pow(linearVelocity_x, 2) + pow(linearVelocity_y, 2));
    if (linear_velocity > this->MAX_LINEAR_VELOCITY || linear_velocity < this->MIN_LINEAR_VELOCITY){
        std::cerr << "Robot " << this->robotID_ << " linear_velocity OUT OF RANGE!!!" << std::endl;
        return false;
    }
    else{
        this->linearVelocity_x_ = linearVelocity_x;
        this->linearVelocity_y_ = linearVelocity_y;
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


