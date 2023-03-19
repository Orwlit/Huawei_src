//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

#include "context.h"

Context::Context() {
    this->dt = 0.0f;
    this->OK_ = "OK";
    this->robotTotalNum_ = 4;
    this->MINIMUM_EQUAL_VALUE_ = 0.001f;
    this->MAX_HZ = 50;

    this->currentMoney_ = 0;
    this->factoryTotalNum_ = 0;

    this->frameID_ = 0;
    this->previousFrameID_ = 0;

    std::cout << "OK" << std::flush;
    this->SYSTEM_ENABLE_ = true;

    if(!(this->Initialize())){
        std::cerr << "Initialization Failed" << std::endl;
    }
    std::cerr << "Initialization complete" << std::endl;
}

bool Context::UpdateAllStatus() {
    std::cin >> this->frameID_ >> this->currentMoney_;
    if (std::cin.bad()) {
        std::cerr << "UPDATE FRAME_ID or CURRENT_MONEY FAILED!!!" << std::endl;
        std::cerr << "frameID_: " << this->frameID_ << "\t"
                  << "currentMoney_: " << this->currentMoney_ << std::endl;
        return false;
    }

//    std::cerr << "frameID_: " << this->frameID_ << "\t" << "UpdateAllStatus start!!!" << std::endl;

    std::cin >> this->factoryTotalNum_;
    if (std::cin.bad()) {
        std::cerr << "UPDATE FactoryTotalNum FAILED!!!" << std::endl;
        std::cerr << "factoryTotalNum_: " << factoryTotalNum_ << std::endl;
        return false;
    }

    int factoryType_cache;
    int remainingFrame_cache;
    int warehouseStatus_cache;
    int productStatus_cache;
    float factory_x_input, factory_y_input;
    for (int factory_index = 0; factory_index < this->factoryTotalNum_; ++factory_index) {
//        std::cerr << "---------------Reading Factory Info---------------" << std::endl;
        //读取对应的工厂信息
        std::cin >> factoryType_cache;
        if (std::cin.bad()) {
            std::cerr << "FACTORY Type READ FAILED!!!" << std::endl;
            std::cerr << "factoryType: " << factoryType_cache << std::endl;
            return false;
        }
//        std::cerr << "factoryType: " << factoryType_cache << std::endl;

        std::cin >> factory_x_input >> factory_y_input;
        if (std::cin.bad()) {
            std::cerr << "FACTORY Pose READ FAILED!!!" << std::endl;
            std::cerr << "x: " << factory_x_input << "\ty: " << factory_y_input << std::endl;
            return false;
        }
//        std::cerr << "x: " << factory_x_input << "\ty: " << factory_y_input << std::endl;

        std::cin >> remainingFrame_cache;
        if (std::cin.bad()) {
            std::cerr << "FACTORY Remaining Time READ FAILED!!!" << std::endl;
            std::cerr << "remainingTime: " << remainingFrame_cache << std::endl;
            return false;
        }
//        std::cerr << "remainingTime: " << remainingFrame_cache << std::endl;
        std::cin >> warehouseStatus_cache >> productStatus_cache;
        if (std::cin.bad()) {
            std::cerr << "FACTORY Space Status READ FAILED!!!" << std::endl;
            std::cerr << "warehouseStatus: " << warehouseStatus_cache << std::endl;
            std::cerr << "productStatus: " << productStatus_cache << std::endl;
            return false;
        }
//        std::cerr << "warehouseStatus: " << warehouseStatus_cache << std::endl;
//        std::cerr << "productStatus: " << productStatus_cache << std::endl;


//        std::cerr << "--------------Updating Factory NO." << factory_index << " info---------------" << std::endl;
        //更新对应的工厂信息 根据坐标查找对应工厂
//        bool isUpdate = false;
        //刷新工厂类型，希望不刷新。若刷新了，则打印错误信息到cerr
        if (this->allFactories_[factory_index]->SetType(static_cast<FactoryType>(factoryType_cache))){
            std::cerr << "FactoryID: " << factory_index << " has CHANGED TYPE!!!" << std::endl;
        }
        //刷新其他信息
        if (!(this->allFactories_[factory_index]->SetRemainingFrame(remainingFrame_cache))){
            std::cerr << "FactoryID: " << factory_index << " SetRemainingFrame FAILED" << std::endl;
        }
        if (!(this->allFactories_[factory_index]->SetWarehouseStatus(warehouseStatus_cache))){
            std::cerr << "FactoryID: " << factory_index << " SetWarehouseStatus FAILED!!!" << std::endl;
        }
        if (!(this->allFactories_[factory_index]->SetProductStatus(productStatus_cache))){
            std::cerr << "FactoryID: " << factory_index << " SetProductStatus FAILED!!!" << std::endl;
        }
//        isUpdate = true;


//        for (int factory_index = 0; factory_index < this->factoryTotalNum_; ++factory_index) {
//            double distance = 0.0;
//            const float factory_x = this->GetAllFactories()[factory_index]->GetCoordinate()[0];
//            const float factory_y = this->GetAllFactories()[factory_index]->GetCoordinate()[1];
//
//            distance = sqrt(pow((factory_x - factory_x_input), 2) + pow((factory_y - factory_y_input), 2));
//            if (distance < this->MINIMUM_EQUAL_VALUE_){
//                //刷新工厂类型，希望不刷新。若刷新了，则打印错误信息到cerr
//                if (this->allFactories_[factory_index]->SetType(static_cast<FactoryType>(factoryType_cache))){
//                    std::cerr << "FactoryID: " << factory_index << " has CHANGED TYPE!!!" << std::endl;
//                }
//                //刷新其他信息
//                if (!(this->allFactories_[factory_index]->SetRemainingFrame(remainingFrame_cache))){
//                    std::cerr << "FactoryID: " << factory_index << " SetRemainingFrame FAILED" << std::endl;
//                }
//                if (!(this->allFactories_[factory_index]->SetWarehouseStatus(warehouseStatus_cache))){
//                    std::cerr << "FactoryID: " << factory_index << " SetWarehouseStatus FAILED!!!" << std::endl;
//                }
//                if (!(this->allFactories_[factory_index]->SetProductStatus(productStatus_cache))){
//                    std::cerr << "FactoryID: " << factory_index << " SetProductStatus FAILED!!!" << std::endl;
//                }
//                isUpdate = true;
//                break; //找到对应的工厂，且状态更新结束
//            }
//        }

//        if (!isUpdate){
//            std::cerr << "Factory Coordinate (" << factory_x_input << ", " << factory_y_input << ") NOT FOUND !!!" << std::endl;
//        }
    }

    int nearbyFactoryID_cahce; //-1：表示当前没有处于任何工作台附近   [0,工作台总数-1] ：表示某工作台的下标，从0开始，按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
    int carryingType_cahce; //范围[0,7] 0表示未携带物品 1-7表示对应物品
    float timePunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float crashPunishment_cahce; //携带物品时为[0.8, 1]的浮点数，不携带物品时为0
    float angularVelocity_cahce; //角速度
    float linearVelocity_x_cahce, linearVelocity_y_cahce; //线速度 x, y
    float orientation_cahce; //朝向
    float robot_x_input, robot_y_input; //坐标

    //读取对应的机器人信息
    for (int robot_index = 0; robot_index < this->robotTotalNum_; ++robot_index) {
//        std::cerr << "--------------Reading Robot NO." << robot_index << " info---------------" << std::endl;

        std::cin >> nearbyFactoryID_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT nearbyFactoryID READ FAILED!!!" << std::endl;
            std::cerr << "nearbyFactoryID: " << nearbyFactoryID_cahce << std::endl;
            return false;
        }

        std::cin >> carryingType_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT carryingType READ FAILED!!!" << std::endl;
            std::cerr << "carryingType: " << carryingType_cahce << std::endl;
            return false;
        }

        std::cin >> timePunishment_cahce >> crashPunishment_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT Punishment READ FAILED!!!" << std::endl;
            return false;
        }

        std::cin >> angularVelocity_cahce >> linearVelocity_x_cahce >> linearVelocity_y_cahce;
        if (std::cin.bad()) {
            std::cerr << "ROBOT Velocity READ FAILED!!!" << std::endl;
            return false;
        }

        std::cin >> orientation_cahce >> robot_x_input >> robot_y_input;
        if (std::cin.bad()) {
            std::cerr << "ROBOT Pose READ FAILED!!!" << std::endl;
            return false;
        }

        //更新对应的机器人信息
//        std::cerr << "--------------Updating Robot NO." << robot_index << " info---------------" << std::endl;

        if (!(this->allRobots_[robot_index]->SetNearbyFactoryID(nearbyFactoryID_cahce))) {
            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
                      << " SetNearbyFactoryID FAILED!!!\t";
        }
        if (!(this->allRobots_[robot_index]->SetCarryingType(carryingType_cahce))) {
            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
                      << " SetCarryingType FAILED!!!\t";
        }
        if (!(this->allRobots_[robot_index]->SetPunishments(timePunishment_cahce, crashPunishment_cahce))) {
            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
                      << " SetPunishments FAILED!!!\t";
        }
        if (!(this->allRobots_[robot_index]->SetAngularVelocity(angularVelocity_cahce))) {
            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
                      << " SetAngularVelocity FAILED!!!\t";
        }
        if (!(this->allRobots_[robot_index]->SetLinearVelocity(linearVelocity_x_cahce, linearVelocity_y_cahce))) {
//            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
//                      << " SetLinearVelocity FAILED!!!\t";
        }
        if (!(this->allRobots_[robot_index]->SetOrientation(orientation_cahce))) {
            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
                      << " SetOrientation FAILED!!!\t";
        }
        if (!(this->allRobots_[robot_index]->SetCoordinate(robot_x_input, robot_y_input))) {
            std::cerr << "ROBOT NO." << this->allRobots_[robot_index]->GetRobotID()
                      << " SetCoordinate FAILED!!!\t";
        }
    }

    //读取OK，判断信息结束
    std::string ok_get;
    std::cin >> ok_get;
    if (std::cin.bad() || ok_get != this->OK_) {
        std::cerr << "UpdateAllStatus NOT OK!!!" << std::endl;
        std::cerr << "ok_get: " << ok_get << "\tthis->OK_: " << this->OK_ << std::endl;

        return false;
    }

    //接收广播后计算dt，相邻帧时间
    this->dt = (static_cast<float>(this->GetFrameId() - this->GetPreviousFrameId()))
            * (1.0f / static_cast<float>(this->GetMAX_Hz()));
    if (this->dt == 0){
        std::cerr << "dt = 0!!!, frameID_: " << this->GetFrameId()
                  << ", previousFrameID_: " << this->GetPreviousFrameId() << std::endl;
    }
//    std::cerr << "dt: " << this->dt << std::endl;

    return true;
}

bool Context::Initialize() {
    std::vector<std::string> initial_map;
    initial_map.resize(100);
    int robotID = 0;
    int factoryID = 0;
    char current_char;
    for (int col = 0; col < 100; ++col) {
        std::getline(std::cin, initial_map[col]);
        for (int row = 0; row < 100; ++row) {
            current_char = initial_map[col][row];
            if (current_char == '.'){
                continue;
            }
            //计算当前字符对应的坐标
            float x = 0.25f + 0.5f * static_cast<float>(row);
            float y = 50.0f - 0.25f - 0.5f * static_cast<float>(col);

            if (current_char == 'A'){
                //TODO: 为什么emplace_back就不可以？？？
                allRobots_.push_back(std::make_shared<Robot>(robotID, x, y));
//                allRobots_.emplace_back(std::make_shared<Robot>(robotID, x, y));
                ++robotID;
                continue;
            }
            if (isdigit(current_char)){
                int tmp = current_char - '0'; //直接static_cast会让current_char变为ASCII码，50
                auto factoryType = static_cast<FactoryType>(tmp);
                //TODO: 为什么emplace_back就不可以？？？
                this->allFactories_.push_back(std::make_shared<Factory>(factoryID, factoryType, x, y));
//                this->allFactories_.emplace_back(std::make_shared<Factory>(factoryID, factoryType, x, y));
//                allFactories_[factoryID] = std::make_shared<Factory>(factoryID, factoryType, x, y);
//                std::cerr << "factoryID: " << factoryID << " Created, x: " << x << " y: " << y << std::endl;
                ++factoryID;
                continue;
            }
        }
    }



    //读取OK，判断信息结束
    std::string ok_get;
    if (!(std::cin >> ok_get) || ok_get != this->OK_) {
        std::cerr << "Context INITIALIZATION NOT RECEIVE \"OK\"!!!" << std::endl;
        return false;
    }
    return true;
}

int Context::GetFrameId() const {
    return frameID_;
}

int Context::GetPreviousFrameId() const {
    return previousFrameID_;
}

const std::vector<std::shared_ptr<Robot>> &Context::GetAllRobots() const {
    return allRobots_;
}

const std::vector<std::shared_ptr<Factory>> &Context::GetAllFactories() const {
    return allFactories_;
}

int Context::GetMAX_Hz() const {
    return MAX_HZ;
}

int Context::GetCurrentMoney() const {
    return currentMoney_;
}

bool Context::IsSystemEnable() const {
    return SYSTEM_ENABLE_;
}

void Context::SetPreviousFrameId(int previousFrameId) {
    previousFrameID_ = previousFrameId;
}

float Context::DistanceFR(int robotID, int factoryID) const {
    float robot_x = this->GetRobot(robotID)->GetCoordinate()[0];
    float robot_y = this->GetRobot(robotID)->GetCoordinate()[1];
    float factory_x = this->GetFactory(factoryID)->GetCoordinate()[0];
    float factory_y = this->GetFactory(factoryID)->GetCoordinate()[1];
    double distance = sqrt(pow(factory_x - robot_x, 2) + pow(factory_y - robot_y, 2));
    return static_cast<float>(distance);
}

float Context::DistanceFF(int factory1_ID, int factory2_ID) const {
    float factory1_x = this->GetFactory(factory1_ID)->GetCoordinate()[0];
    float factory1_y = this->GetFactory(factory1_ID)->GetCoordinate()[1];
    float factory2_x = this->GetFactory(factory2_ID)->GetCoordinate()[0];
    float factory2_y = this->GetFactory(factory2_ID)->GetCoordinate()[1];
    double distance = sqrt(pow(factory2_x - factory1_x, 2) + pow(factory2_y - factory1_y, 2));
    return static_cast<float>(distance);
}

float Context::DistanceRR(int robot1_ID, int robot2_ID) const {
    float robot1_x = this->GetRobot(robot1_ID)->GetCoordinate()[0];
    float robot1_y = this->GetRobot(robot1_ID)->GetCoordinate()[1];
    float robot2_x = this->GetRobot(robot2_ID)->GetCoordinate()[0];
    float robot2_y = this->GetRobot(robot2_ID)->GetCoordinate()[1];
    double distance = sqrt(pow(robot2_x - robot1_x, 2) + pow(robot2_y - robot1_y, 2));
    return static_cast<float>(distance);
}

std::shared_ptr<Robot> Context::GetRobot(int robotIndex) const {
    return this->GetAllRobots()[robotIndex];
}

std::shared_ptr<Factory> Context::GetFactory(int factoryIndex) const {
    return this->GetAllFactories()[factoryIndex];
}

bool
Context::AboutToCrash(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2, float k) const {
    float robot1LinearVelocity = robot1->GetLinearVelocity();
    float robot2LinearVelocity = robot2->GetLinearVelocity();
    float robot1_dt_velocity = k * this->GetDt() * robot1LinearVelocity;
    float robot2_dt_velocity = k * this->GetDt() * robot2LinearVelocity;
    float orientation1 = robot1->GetOrientation();
    float orientation2 = robot2->GetOrientation();

//    float robot1_dt_vector_x = robot1_dt_velocity * cos(orientation1);
//    float robot1_dt_vector_y = robot1_dt_velocity * sin(orientation1);
//    float robot2_dt_vector_x = robot2_dt_velocity * cos(orientation2);
//    float robot2_dt_vector_y = robot2_dt_velocity * sin(orientation2);

    float vector_x_between = robot2->GetCoordinate()[0] - robot1->GetCoordinate()[0];
    float vector_y_between = robot2->GetCoordinate()[1] - robot1->GetCoordinate()[1];
    float distance_between = sqrt(pow(vector_x_between, 2) + pow(vector_y_between, 2));

//    bool vector_condition_r1_plus_r2 = ((robot1_dt_vector_x + robot2_dt_vector_x) == vector_x_between) &&
//            ((robot1_dt_vector_y + robot2_dt_vector_y) == vector_y_between);
//    bool vector_condition_r1_plus_d = ((robot1_dt_vector_x + vector_x_between) == robot2_dt_vector_x) &&
//                                       ((robot1_dt_vector_y + vector_y_between) == robot2_dt_vector_y);
//    bool vector_condition_r2_plus_d = ((robot2_dt_vector_x + vector_x_between) == robot1_dt_vector_x) &&
//                                       ((robot2_dt_vector_y + vector_y_between) == robot1_dt_vector_y);

    bool triangle_condition_r1_plus_r2 = (robot1_dt_velocity + robot2_dt_velocity) > distance_between;
    bool triangle_condition_r1_plus_d = (robot1_dt_velocity + distance_between) > robot2_dt_velocity;
    bool triangle_condition_r2_plus_d = (robot2_dt_velocity + distance_between) > robot1_dt_velocity;
    bool triangle_condition_r1_minus_r2 = std::abs((robot1_dt_velocity - robot2_dt_velocity)) < std::abs(distance_between);
    bool triangle_condition_r1_minus_d = std::abs((robot1_dt_velocity - distance_between)) < std::abs(robot2_dt_velocity);
    bool triangle_condition_r2_minus_d = std::abs((robot2_dt_velocity - distance_between)) < std::abs(robot1_dt_velocity);

//    bool vector_condition = vector_condition_r1_plus_r2 && vector_condition_r1_plus_d && vector_condition_r2_plus_d;
    bool triangle_condition = (triangle_condition_r1_plus_r2
                            && triangle_condition_r1_plus_d
                            && triangle_condition_r2_plus_d
                            && triangle_condition_r1_minus_r2
                            && triangle_condition_r1_minus_d
                            && triangle_condition_r2_minus_d);

    if (!triangle_condition){
        return false;
    }
    std::cerr << "triangle_condition passed\t";

    // 将1和2间的距离作为新坐标轴正方向，根据行进方向判断是否相撞
    //TODO: 象限判断错误s
    float theta_distance = std::atan2(vector_y_between, vector_x_between);
    float orientation1_new = orientation1 - theta_distance;
    float orientation2_new = orientation2 - theta_distance;
    float left_angle = 0.0f;
    float right_angle = 0.0f;

    if (robot1->GetCoordinate()[0] > robot2->GetCoordinate()[0]){
        left_angle = orientation1_new;
        right_angle = orientation2_new;
        std::cerr << "R1_LEFT_angle: " << 180 * orientation1_new / M_PI << ", R2_RIGHT_angle: " << orientation2_new << std::endl;
    }
    else{
        left_angle = orientation2_new;
        right_angle = orientation1_new;
        std::cerr << "R2_LEFT_angle: " << 180 * orientation2_new / M_PI << ", R1_RIGHT_angle: " << orientation1_new << std::endl;
    }

    // 根据象限判断是否有相撞的可能。左1象限右2象限 或 左4象限右3象限
    bool condition_left1_right2 = (left_angle >= 0 && left_angle <= M_PI / 2) && (right_angle >= M_PI / 2 && right_angle <= M_PI);
    bool condition_left4_right3 = (left_angle >= -M_PI/2 && left_angle <= 0) && (right_angle >= -M_PI && right_angle <= -M_PI/2);

//    if (!condition_left1_right2 || !condition_left4_right3){
//        return false;
//    }

    std::cerr << "CRASH WARNING!!!" << std::endl;
    return true;
}

float Context::GetDt() const {
    return dt;
}

bool Context::UpdateGraph() {
    return false;
}




