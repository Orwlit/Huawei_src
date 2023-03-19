//
// Created by orw on 3/14/23.
//
#ifndef HUAWEI_FACTORY_H
#define HUAWEI_FACTORY_H

#include <vector>


enum FactoryType{
    UNKNOWN = 0, //一开始均设为UNKNOWN
    MATERIAL_1 = 1,
    MATERIAL_2 = 2,
    MATERIAL_3 = 3,
    FACTORY_4 = 4,
    FACTORY_5 = 5,
    FACTORY_6 = 6,
    FACTORY_7 = 7,
    SELLER_8 = 8,
    SELLER_9 = 9
};

enum FactoryFlag{
    READY = 0,
    PRODUCING = 1,
    VACANT = 2
};

class Factory{
private:

    int factoryID_;
    FactoryType factoryType_;
    FactoryFlag factoryFlag_;
    float coordinate_[2];
    int remainingFrame_; //剩余生产帧数: -1表示没有生产 0表示生产因输出格满而阻塞 >=0表示剩余生产帧数
    int warehouseState_; //仓库格状态
    int productState_; //产品格状态

    int MAX_REMAINING_FRAME;
public:
    Factory();
    Factory(int factoryID, FactoryType factoryType, float x_initial, float y_initial);;

    //Getter
    [[nodiscard]] FactoryFlag GetFactoryFlag();
    [[nodiscard]] FactoryType GetFactoryType();
    [[nodiscard]] const float* GetCoordinate() const;
    [[nodiscard]] int GetFactoryId() const;
    [[nodiscard]] int GetRemainingFrame() const;
    [[nodiscard]] int GetWarehouseState() const;
    [[nodiscard]] int GetProductState() const;

    bool SetType(FactoryType type);
    bool SetFlag(FactoryFlag flag);
    bool SetRemainingFrame(int time);
    bool SetWarehouseStatus(int state);
    bool SetProductStatus(int state);


};

#endif //HUAWEI_FACTORY_H
