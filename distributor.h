//
// Created by orw on 3/14/23.
//

#ifndef HUAWEI_DISTRIBUTOR_H
#define HUAWEI_DISTRIBUTOR_H
#pragma once


enum DistributorFlag{
    RARE = 0, //有9时才行，123优先
    MEDIUM_RARE = 1,
    MEDIUM = 2, //456优先
    MEDIUM_WELL = 3,
    WELL_DONE = 4 //7优先
};

class Distributor{
public:
    Distributor();

private:



};
#endif //HUAWEI_DISTRIBUTOR_H
