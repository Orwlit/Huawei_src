//
// Created by orw on 3/14/23.
//
#include <iostream>

#include "factory.h"

Factory::Factory() {
    this->factoryType_ = FactoryType::UNKNOWN;
}

bool Factory::SetType(FactoryType type) {
    if (this->factoryType_ != type){
        std::cerr << "old type: " << this->factoryType_ << "\t new type: " << type << std::endl;
        return true;
    }
    return false;
}

bool Factory::SetFlag(FactoryFlag flag) {
    if (this->factoryFlag_ != flag){
        this->factoryFlag_ = flag;
        return true;
    }
    return false;
}

float* Factory::GetCoordinate() {
    return this->coordinate_;
}

FactoryFlag Factory::GetFlag() {
    return this->factoryFlag_;
}

FactoryType Factory::GetType() {
    return this->factoryType_;
}

bool Factory::SetRemainingFrame(int time) {
    if (time > this->MAX_REMAINING_FRAME){
        std::cerr << "FactoryID: " << this->factoryID_
                  << "\tSetRemainingFrame OUT OF RANGE, MAX_REMAINING_FRAME: " << this->MAX_REMAINING_FRAME << std::endl;
        return false;
    }
    this->remainingFrame_ = time;
    return true;
}

bool Factory::SetWarehouseStatus(int state) {
    if (state < 0){
        std::cerr << "FactoryID: " << this->factoryID_
                  << "\tSetWarehouseStatus OUT OF RANGE, should be 0 or 1: " << std::endl;
        return false;
    }
    this->warehouseState_ = state;
    return true;
}

bool Factory::SetProductStatus(int state) {
    if (state != 0 && state != 1){
        std::cerr << "FactoryID: " << this->factoryID_
                  << "\tSetProductStatus OUT OF RANGE, should be 0 or 1: " << std::endl;
        return false;
    }
    this->productState_ = state;
    return true;
}

Factory::Factory(int factoryID, FactoryType factoryType, float x_initial, float y_initial) {
    this->factoryID_ = factoryID;
    this->factoryType_ = factoryType;
    this->coordinate_[0] = x_initial;
    this->coordinate_[1] = y_initial;

    this->MAX_REMAINING_FRAME = 1000; //7号的时间为1000帧
}


