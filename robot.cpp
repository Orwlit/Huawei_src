//
// Created by orw on 3/14/23.
//
#include <iostream>
#include "robot.h"

Robot::Robot() {

}

void Robot::Forward(float speed) {
    std::cout << "forward" << " " << robotID_ << " " << speed << std::endl;
}

void Robot::Rotate(float speed) {
    std::cout << "rotate" << " " << robotID_ << " " << speed << std::endl;
}

void Robot::Buy(){
    std::cout << "buy" << " " << robotID_ << std::endl;
};

void Robot::Sell(){
    std::cout << "sell" << " " << robotID_ << std::endl;
};

void Robot::Destroy(){
    std::cout << "destroy" << " " << robotID_ << std::endl;
};


void Robot::TrickLoop() {

}

void Robot::HighSpeedMove() {

}

void Robot::LowSpeedMove() {

}
