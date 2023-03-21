//
// Created by orw on 3/14/23.
//
#ifndef HUAWEI_FACTORY_H
#define HUAWEI_FACTORY_H

#include <vector>
#include <map>
#include <set>

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

enum FactoryClass{
    A = 1, //123
    B = 2, //456
    C = 3, //7
    D = 4  //89
};

enum FactoryFlag{
    READY = 0,
    PRODUCING = 1,
    VACANT = 2
};

class Factory{
private:
    std::map<int, int> warehouseMap;

    int factoryID_;
    FactoryType factoryType_;
    FactoryClass factoryClass_;
    FactoryFlag factoryFlag_;

    float coordinate_[2];
    int remainingFrame_; //剩余生产帧数: -1表示没有生产 0表示生产因输出格满而阻塞 >=0表示剩余生产帧数
    std::map<FactoryType, bool> warehouseState_; //仓库格状态
    std::set<FactoryType> warehouseType_;
//    std::vector<std::pair<int, int>> warehouseState_
    int productState_; //产品格状态

    int MAX_REMAINING_FRAME;
public:
    Factory();
    Factory(int factoryID, FactoryType factoryType, float x_initial, float y_initial);;

    //Getter
    [[nodiscard]] FactoryFlag GetFactoryFlag() const;
    [[nodiscard]] FactoryType GetFactoryType() const;
    [[nodiscard]] FactoryClass GetFactoryClass() const;
    [[nodiscard]] const float* GetCoordinate() const;
    [[nodiscard]] int GetFactoryId() const;
    [[nodiscard]] int GetRemainingFrame() const;
    [[nodiscard]] std::map<FactoryType, bool> GetWarehouseState() const;
    [[nodiscard]] int GetProductState() const;
    [[nodiscard]] const std::set<FactoryType> &GetWarehouseType() const;

    //Setter
    void SetFactoryClass(FactoryClass factoryClass);
    bool SetType(FactoryType type);
    bool SetFlag(FactoryFlag flag);
    bool SetRemainingFrame(int time);
//    void SetWarehouseType(FactoryType warehouseType);
    bool SetWarehouseState(const std::map<FactoryType, bool> &warehouseState);
    void SetWarehouseState(FactoryType type, bool state);
    void SetWarehouseType(const std::set<FactoryType> &warehouseType);
    //    bool SetWarehouseState(FactoryType type, bool state);
//    bool SetWarehouseState(std::set<FactoryType> warehouseType);
    bool SetProductStatus(int state);


};

#endif //HUAWEI_FACTORY_H
