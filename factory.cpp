//
// Created by orw on 3/14/23.
//

#include "factory.h"

Factory::Factory() {
    this->type_ = FactoryType::UNKNOWN;
}

bool Factory::setType(int type) {
    if (this->type_ != type){
        return true;
    }
    return false;
}


