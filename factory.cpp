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

const float* Factory::GetCoordinate() const {
    return this->coordinate_;
}

FactoryFlag Factory::GetFactoryFlag() const {
    return this->factoryFlag_;
}

FactoryType Factory::GetFactoryType() const {
    return this->factoryType_;
}

bool Factory::SetRemainingFrame(int time) {
    if (time > this->MAX_REMAINING_FRAME_){
        std::cerr << "FactoryID: " << this->factoryID_
                  << "\tSetRemainingFrame OUT OF RANGE, MAX_REMAINING_FRAME_: " << this->MAX_REMAINING_FRAME_ << std::endl;
        return false;
    }
    this->remainingFrame_ = time;
    return true;
}

bool Factory::SetWarehouseState(const std::map<FactoryType, std::pair<bool, bool>> &warehouseState) {
    warehouseState_ = warehouseState;
    return true;
}

void Factory::SetWarehouseState(FactoryType type, bool state) {
    this->warehouseState_[type].first = state;
}

void Factory::SetWarehouseFlag(FactoryType type, bool flag) {
    this->warehouseState_[type].second = flag;
}

bool Factory::SetProductStatus(bool state) {
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

    this->MAX_REMAINING_FRAME_ = 1000; //7号的时间为1000帧
}

int Factory::GetFactoryId() const {
    return factoryID_;
}

int Factory::GetRemainingFrame() const {
    return remainingFrame_;
}

std::map<FactoryType, std::pair<bool, bool>> Factory::GetWarehouseState() const {
//    for (auto it : warehouseState_) {
//        std::cerr << "Key: " << it.first << " Value1: " << it.second.first << " Value2: " << it.second.second << std::endl;
//    }

    return this->warehouseState_;
}

bool Factory::GetProductState() const {
    return productState_;
}

FactoryClass Factory::GetFactoryClass() const {
    return factoryClass_;
}

void Factory::SetFactoryClass(FactoryClass factoryClass) {
    this->factoryClass_ = factoryClass;
}

const std::set<FactoryType> &Factory::GetWarehouseType() const {
    return warehouseType_;
}

void Factory::SetWarehouseType(const std::set<FactoryType> &warehouseType) {
    this->warehouseType_ = warehouseType;
}



