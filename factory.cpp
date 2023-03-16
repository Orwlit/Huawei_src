//
// Created by orw on 3/14/23.
//

#include "factory.h"

Factory::Factory() {
    this->factoryType_ = FactoryType::UNKNOWN;
}

bool Factory::SetType(FactoryType type) {
    if (this->factoryType_ != type){
        return true;
    }
    return false;
}

float* Factory::GetCoordinate() {
    return this->coordinate_;
}

bool Factory::SetRemainingTime(int time) {
    this->remainingTime_ = time;
    return true;
}

bool Factory::SetWarehouseStatus(int state) {
    this->warehouseState_ = state;
    return false;
}

bool Factory::SetProductStatus(int state) {
    this->productState_ = state;
    return false;
}

Factory::Factory(FactoryType factoryType, float x_initial, float y_initial) {
    this->factoryType_ = factoryType;
    this->coordinate_[0] = x_initial;
    this->coordinate_[1] = y_initial;
}


