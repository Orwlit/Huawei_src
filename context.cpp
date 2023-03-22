//
// Created by orw on 3/14/23.
//
#include "context.h"

Context::Context() {
    this->dt = 0.0f;

    this->currentMoney_ = 0;
    this->factoryTotalNum_ = 0;
    this->frameID_ = 0;
    this->previousFrameID_ = 0;
    this->SYSTEM_ENABLE_ = true;

    // 初始化，主要完成Factory对象的信息初始化
    if(!(this->Initialize())){
        std::cerr << "Initialization Failed" << std::endl;
    }
    std::cerr << "Initialization complete" << std::endl;

    this->nodeTotalNum_ = this->factoryTotalNum_ + this->robotTotalNum_;

    // 创建历史图，并检查是否满足对称矩阵条件
    if (!(this->GenerateHistoryGraph())){
        std::cerr << "GenerateHistoryGraph in Context::Context() ERROR!!!" << std::endl;
    }

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

    int factoryTotalNumCache = 0;
    std::cin >> factoryTotalNumCache;
    if (std::cin.bad()) {
        std::cerr << "UPDATE FactoryTotalNum FAILED!!!" << std::endl;
        std::cerr << "factoryTotalNum_: " << factoryTotalNum_ << std::endl;
        return false;
    }
    if (factoryTotalNumCache != this->factoryTotalNum_){
        std::cerr << "factoryTotalNum_ calculate ERROR in Context::initialize(), factoryTotalNum_ = " << this->factoryTotalNum_
        << ", should be: " << factoryTotalNumCache << std::endl;
    }

    int factoryType_cache;
    int remainingFrame_cache;
    int warehouseState_raw_cache;
    bool productStatus_cache;
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
//        std::cerr << factoryType_cache << std::endl;
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
        std::cin >> warehouseState_raw_cache >> productStatus_cache;
        if (std::cin.bad()) {
            std::cerr << "FACTORY Space Status READ FAILED!!!" << std::endl;
            std::cerr << "warehouseStatus: " << warehouseState_raw_cache << std::endl;
            std::cerr << "productStatus: " << productStatus_cache << std::endl;
            return false;
        }
//        std::cerr << "warehouseStatus: " << warehouseState_raw_cache << std::endl;
//        std::cerr << "productStatus: " << productStatus_cache << std::endl;


//        std::cerr << "--------------Updating Factory NO." << factory_index << " info---------------" << std::endl;
        //更新对应的工厂信息 根据坐标查找对应工厂
//        bool isUpdate = false;
        //刷新工厂类型，希望不刷新。若刷新了，则打印错误信息到cerr
        if (this->allFactories_[factory_index]->SetType(static_cast<FactoryType>(factoryType_cache))) {
            std::cerr << "FactoryID: " << factory_index << " has CHANGED TYPE!!!" << std::endl;
        }

        //刷新其他信息
        if (!(this->allFactories_[factory_index]->SetRemainingFrame(remainingFrame_cache))) {
            std::cerr << "FactoryID: " << factory_index << " SetRemainingFrame FAILED" << std::endl;
        }

        FactoryType tmp_type = this->GetFactory(factory_index)->GetFactoryType();
        auto warehouseState_cache = WarehouseStateConversion(tmp_type, warehouseState_raw_cache);
        if (!(this->allFactories_[factory_index]->SetWarehouseState(warehouseState_cache))) {
            std::cerr << "FactoryID: " << factory_index << " SetWarehouseState FAILED!!!" << std::endl;
        }

        FactoryClass current_factory_class = this->GetFactory(factory_index)->GetFactoryClass();
        // 认为A类工厂始终有产品
        if (current_factory_class == FactoryClass::A){
            this->allFactories_[factory_index]->SetProductStatus(true);
        } else if (!(this->allFactories_[factory_index]->SetProductStatus(productStatus_cache))) {
            std::cerr << "FactoryID: " << factory_index << " SetProductStatus FAILED!!!" << std::endl;
        }
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
                allRobots_.push_back(std::make_shared<Robot>(robotID, x, y));
                ++robotID;
                continue;
            }
            if (isdigit(current_char)){
                int tmp = current_char - '0'; //直接static_cast会让current_char变为ASCII码，50
                auto factoryType = static_cast<FactoryType>(tmp);
                this->allFactories_.push_back(std::make_shared<Factory>(factoryID, factoryType, x, y));
//                std::cerr << "factoryID: " << factoryID << " Created, x: " << x << " y: " << y << std::endl;

                // 初始化工厂类型
                this->allFactories_[factoryID]->SetType(factoryType);

                // 初始化仓库状态
                std::set<FactoryType> warehouseType;
                std::map<FactoryType, std::pair<bool, bool>> warehouseState;
                int factory_index_shift = this->GetRobotTotalNum();
                switch (factoryType) {
                    case FactoryType::UNKNOWN:
                        std::cerr << "Factory NO." << this->allFactories_[factoryID]->GetFactoryId() << "type is UNKNOWN!!!" << std::endl;
                        break;

                    case FactoryType::MATERIAL_1:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::A);
                        oneFactories_.push_back(this->allFactories_[factoryID]);
                        this->oneFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_4][MATERIAL_1].push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_5][MATERIAL_1].push_back(factoryID);

                        warehouseType.insert(UNKNOWN);
                        warehouseState[UNKNOWN].first = false;
                        warehouseState[UNKNOWN].second = false;
                        break;
                    case FactoryType::MATERIAL_2:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::A);
                        twoFactories_.push_back(this->allFactories_[factoryID]);
                        this->twoFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_4][MATERIAL_2].push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_6][MATERIAL_2].push_back(factoryID);

                        warehouseType.insert(UNKNOWN);
                        warehouseState[UNKNOWN].first = false;
                        warehouseState[UNKNOWN].second = false;
                        break;
                    case FactoryType::MATERIAL_3:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::A);
                        threeFactories_.push_back(this->allFactories_[factoryID]);
                        this->threeFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_5][MATERIAL_3].push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_6][MATERIAL_3].push_back(factoryID);
                        this->allFactories_[factoryID]->SetWarehouseState(UNKNOWN, false);

                        warehouseType.insert(UNKNOWN);
                        warehouseState[UNKNOWN].first = false;
                        warehouseState[UNKNOWN].second = false;
                        break;

                    case FactoryType::FACTORY_4:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::B);
                        fourFactories_.push_back(this->allFactories_[factoryID]);
                        this->fourFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_7][FACTORY_4].push_back(factoryID);
                        // 先不考虑456卖到9?
                        this->globalFactoryTypeMap_[SELLER_9][FACTORY_4].push_back(factoryID);

                        warehouseType.insert(MATERIAL_1);
                        warehouseState[MATERIAL_1].first = false;
                        warehouseState[MATERIAL_1].second = false;
                        warehouseType.insert(MATERIAL_2);
                        warehouseState[MATERIAL_2].first = false;
                        warehouseState[MATERIAL_2].second = false;
                        break;
                    case FactoryType::FACTORY_5:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::B);
                        fiveFactories_.push_back(this->allFactories_[factoryID]);
                        this->fiveFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_7][FACTORY_5].push_back(factoryID);
                        // 先不考虑456卖到9?
                        this->globalFactoryTypeMap_[SELLER_9][FACTORY_5].push_back(factoryID);

                        warehouseType.insert(MATERIAL_1);
                        warehouseState[MATERIAL_1].first = false;
                        warehouseState[MATERIAL_1].second = false;
                        warehouseType.insert(MATERIAL_3);
                        warehouseState[MATERIAL_3].first = false;
                        warehouseState[MATERIAL_3].second = false;
                        break;
                    case FactoryType::FACTORY_6:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::B);
                        sixFactories_.push_back(this->allFactories_[factoryID]);
                        this->sixFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[FACTORY_7][FACTORY_6].push_back(factoryID);
                        // 先不考虑456卖到9?
                        this->globalFactoryTypeMap_[SELLER_9][FACTORY_6].push_back(factoryID);

                        warehouseType.insert(MATERIAL_2);
                        warehouseState[MATERIAL_2].first = false;
                        warehouseState[MATERIAL_2].second = false;
                        warehouseType.insert(MATERIAL_3);
                        warehouseState[MATERIAL_3].first = false;
                        warehouseState[MATERIAL_3].second = false;
                        break;

                    case FactoryType::FACTORY_7:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::C);
                        sevenFactories_.push_back(this->allFactories_[factoryID]);
                        this->sevenFactoriesIndex_.push_back(factoryID);
                        this->globalFactoryTypeMap_[SELLER_8][FACTORY_7].push_back(factoryID);
                        this->globalFactoryTypeMap_[SELLER_9][FACTORY_7].push_back(factoryID);

                        warehouseType.insert(FACTORY_4);
                        warehouseState[FACTORY_4].first = false;
                        warehouseState[FACTORY_4].second = false;
                        warehouseType.insert(FACTORY_5);
                        warehouseState[FACTORY_5].first = false;
                        warehouseState[FACTORY_5].second = false;
                        warehouseType.insert(FACTORY_6);
                        warehouseState[FACTORY_6].first = false;
                        warehouseState[FACTORY_6].second = false;
                        break;

                    case FactoryType::SELLER_8:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::D);
                        eightFactories_.push_back(this->allFactories_[factoryID]);
                        this->eightFactoriesIndex_.push_back(factoryID);

                        warehouseType.insert(FACTORY_7);
                        warehouseState[FACTORY_7].first = false;
                        warehouseState[FACTORY_7].second = false;
                        break;
                    case FactoryType::SELLER_9:
                        this->allFactories_[factoryID]->SetFactoryClass(FactoryClass::D);
                        nineFactories_.push_back(this->allFactories_[factoryID]);
                        this->nineFactoriesIndex_.push_back(factoryID);

                        warehouseType.insert(FACTORY_4);
                        warehouseState[FACTORY_4].first = false;
                        warehouseState[FACTORY_4].second = false;
                        warehouseType.insert(FACTORY_5);
                        warehouseState[FACTORY_5].first = false;
                        warehouseState[FACTORY_5].second = false;
                        warehouseType.insert(FACTORY_6);
                        warehouseState[FACTORY_6].first = false;
                        warehouseState[FACTORY_6].second = false;
                        warehouseType.insert(FACTORY_7);
                        warehouseState[FACTORY_7].first = false;
                        warehouseState[FACTORY_7].second = false;
                        break;
                }
                this->allFactories_[factoryID]->SetWarehouseType(warehouseType);
                this->allFactories_[factoryID]->SetWarehouseState(warehouseState);


//                //打印检查warehouseType和warehouseState
//                std::cerr << "Factory NO." << this->GetFactory(factoryID)->GetFactoryId()
//                          << " Factory type: " << this->GetFactory(factoryID)->GetFactoryType() << "\twarehouseType_: ";
//                for (auto it : this->GetFactory(factoryID)->GetWarehouseType()) {
//                    std::cerr << it << " ";
//                }
//                std::cerr << "\t";
//                auto myMap = this->GetFactory(factoryID)->GetWarehouseState();
//                for (auto it: myMap)
//                {
//                    std::cerr << "Key: " << it.first << ", Value: " << it.second << " ";
//                }
//                std::cerr << std::endl;

                ++factoryID;
                continue;
            }
        }
    }

    this->factoryTotalNum_ = factoryID;



    //读取OK，判断信息结束
    std::string ok_get;
    if (!(std::cin >> ok_get) || ok_get != this->OK_) {
        std::cerr << "Context INITIALIZATION NOT RECEIVE \"OK\"!!!" << std::endl;
        return false;
    }
    return true;
}

bool Context::GenerateHistoryGraph() {
    // 初始全置为无穷大
    initialHistoryGraph_.resize(this->nodeTotalNum_);
    std::vector<double> infinite_vector;
    infinite_vector.resize(this->nodeTotalNum_);
    for (int i = 0; i < this->nodeTotalNum_; ++i) {
        initialHistoryGraph_[i] = infinite_vector;
    }
    for (int i = 0; i < this->nodeTotalNum_; ++i) {
        for (int j = 0; j < this->nodeTotalNum_; ++j) {
            this->initialHistoryGraph_[i][j] = this->INFINITE_;
        }
    }

    // 机器人到除了89的权值设为距离
    for (int robot_index = 0; robot_index < this->robotTotalNum_; ++robot_index) {
        for (int factory_index = 0; factory_index < this->factoryTotalNum_; ++factory_index) {
            int factory_index_shift = factory_index + this->factoryIDShift_;
            FactoryType currentType = this->GetFactory(factory_index)->GetFactoryType();
            if ((currentType == SELLER_8) || (currentType == SELLER_9)){
                continue;
            } else{
                double robot_x = this->GetRobot(robot_index)->GetCoordinate()[0];
                double robot_y = this->GetRobot(robot_index)->GetCoordinate()[1];
                double factory_x = this->GetFactory(factory_index)->GetCoordinate()[0];
                double factory_y = this->GetFactory(factory_index)->GetCoordinate()[1];
                double distance = sqrt(pow(factory_x - robot_x, 2) + pow(factory_y - robot_y, 2));

                this->initialHistoryGraph_[robot_index][factory_index_shift] = distance;
                this->initialHistoryGraph_[factory_index_shift][robot_index] = distance;
            }
        }
    }

    // 工厂间的距离设为相应距离d
    for (int factory_from_index = 0; factory_from_index < this->factoryTotalNum_; ++factory_from_index) {
        double factory_from_x = this->GetFactory(factory_from_index)->GetCoordinate()[0];
        double factory_from_y = this->GetFactory(factory_from_index)->GetCoordinate()[1];
        FactoryType factory_from_type = this->GetFactory(factory_from_index)->GetFactoryType();
        std::vector<int> factories_to;
        for (const auto& factory_to : this->globalFactoryTypeMap_[factory_from_type]) {
            for (auto factory_to_index : factory_to.second) {
                double factory_to_x = this->GetFactory(factory_to_index)->GetCoordinate()[0];
                double factory_to_y = this->GetFactory(factory_to_index)->GetCoordinate()[1];
                double distance = sqrt(pow(factory_to_x - factory_from_x, 2) + pow(factory_to_y - factory_from_y, 2));
                this->initialHistoryGraph_[factory_from_index + this->factoryIDShift_][factory_to_index + this->factoryIDShift_] = distance;
                this->initialHistoryGraph_[factory_to_index + this->factoryIDShift_][factory_from_index + this->factoryIDShift_] = distance;
            }
        }

    }

    // 检查initialHistoryGraph_是否为对称矩阵，且维数为(nodeTotalNum_)x(nodeTotalNum_)
    bool SymmetricMatrixCheck = true;
    for (int i = 0; i < this->initialHistoryGraph_.size(); ++i) {
        for (int j = 0; j < this->initialHistoryGraph_[i].size(); ++j) {
            if (initialHistoryGraph_[i][j] != initialHistoryGraph_[j][i]){
                std::cerr << "initialHistoryGraph_ NOT a Symmetric Matrix !!!" << std::endl;
                return false;
            }
        }
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

double Context::DistanceFR(int robotID, int factoryID) const {
    float robot_x = this->GetRobot(robotID)->GetCoordinate()[0];
    float robot_y = this->GetRobot(robotID)->GetCoordinate()[1];
    float factory_x = this->GetFactory(factoryID)->GetCoordinate()[0];
    float factory_y = this->GetFactory(factoryID)->GetCoordinate()[1];
    double distance = sqrt(pow(factory_x - robot_x, 2) + pow(factory_y - robot_y, 2));
    return distance;
}

double Context::DistanceFF(int factory1_ID, int factory2_ID) const {
    float factory1_x = this->GetFactory(factory1_ID)->GetCoordinate()[0];
    float factory1_y = this->GetFactory(factory1_ID)->GetCoordinate()[1];
    float factory2_x = this->GetFactory(factory2_ID)->GetCoordinate()[0];
    float factory2_y = this->GetFactory(factory2_ID)->GetCoordinate()[1];
    double distance = sqrt(pow(factory2_x - factory1_x, 2) + pow(factory2_y - factory1_y, 2));
    return distance;
}

double Context::DistanceRR(int robot1_ID, int robot2_ID) const {
    float robot1_x = this->GetRobot(robot1_ID)->GetCoordinate()[0];
    float robot1_y = this->GetRobot(robot1_ID)->GetCoordinate()[1];
    float robot2_x = this->GetRobot(robot2_ID)->GetCoordinate()[0];
    float robot2_y = this->GetRobot(robot2_ID)->GetCoordinate()[1];
    double distance = sqrt(pow(robot2_x - robot1_x, 2) + pow(robot2_y - robot1_y, 2));
    return distance;
}

std::shared_ptr<Robot> Context::GetRobot(int robotIndex) const {
    return this->GetAllRobots()[robotIndex];
}

std::shared_ptr<Factory> Context::GetFactory(int factoryIndex) const {
    return this->GetAllFactories()[factoryIndex];
}

bool
Context::AboutToCrash(const std::shared_ptr<Robot> &robot1, const std::shared_ptr<Robot> &robot2, float k) const {
    float robot1LinearVelocity = robot1->GetLinearVelocity();
    float robot2LinearVelocity = robot2->GetLinearVelocity();
    float robot1_dt_velocity = k * this->GetDt() * robot1LinearVelocity;
    float robot2_dt_velocity = k * this->GetDt() * robot2LinearVelocity;
    float orientation1 = robot1->GetOrientation();
    float orientation2 = robot2->GetOrientation();

    float robot1_x = robot1->GetCoordinate()[0];
    float robot1_y = robot1->GetCoordinate()[1];
    float robot2_x = robot2->GetCoordinate()[0];
    float robot2_y = robot2->GetCoordinate()[1];

    float vector_x_between = robot2_x - robot1_x;
    float vector_y_between = robot2_y - robot1_y;
    float distance_between = sqrt(pow(vector_x_between, 2) + pow(vector_y_between, 2));

    float robot1_head_x = robot1->GetLinearVelocityX() * k * dt + robot1_x;
    float robot1_head_y = robot1->GetLinearVelocityY() * k * dt + robot1_y;
    float robot2_head_x = robot2->GetLinearVelocityX() * k * dt + robot2_x;
    float robot2_head_y = robot2->GetLinearVelocityY() * k * dt + robot2_y;
    float distance_head = sqrt(pow(robot2_head_x - robot1_head_x, 2) + pow(robot2_head_y - robot1_head_y, 2));

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
//    std::cerr << "triangle_condition passed!!!" << std::endl;

//    std::cerr << "d_tail - d_head: " << distance_between - distance_head << std::endl;
//    if (distance_between < distance_head){
//        return false;
//    }
    float condition = distance_between - distance_head;
    if (distance_between > distance_head){

        std::cerr << "CRASH WARNING!!!" << std::endl;
        return true;
    }

    return false;


    // 将1和2间的距离作为新坐标轴正方向，根据行进方向判断是否相撞
    float theta_distance = std::atan2(vector_y_between, vector_x_between);
    float orientation1_new = orientation1 - theta_distance;
    float orientation2_new = orientation2 - theta_distance;
//    std::cerr << "theta_distance: " <<  180 * theta_distance / M_PI
//    << " | orientation1_old: " <<  180 * orientation1 / M_PI
//    << " | orientation2_old: " <<  180 * orientation2 / M_PI << std::endl;
    float left_angle = 0.0f;
    float right_angle = 0.0f;

    if (robot1->GetCoordinate()[0] < robot2->GetCoordinate()[0]){
        left_angle = orientation1_new;
        right_angle = orientation2_new;
//        std::cerr << "LEFT is " << robot1->GetRobotID() << "\tRIGHT is " << robot2->GetRobotID() << std::endl;
    }
    else{
        left_angle = orientation2_new;
        right_angle = orientation1_new;
//        std::cerr << "LEFT is " << robot1->GetRobotID() << "\tRIGHT is " << robot2->GetRobotID() << std::endl;
    }
//    std::cerr << "LEFT_angle: " << 180 * left_angle / M_PI << ", RIGHT_angle: " << 180 * right_angle / M_PI << std::endl;

    // 根据象限判断是否有相撞的可能。左1象限右2象限 或 左4象限右3象限
    bool left1 = left_angle >= 0 && left_angle <= M_PI / 2;
    bool left4 = left_angle >= -M_PI/2 && left_angle <= 0;
    bool right2 = right_angle >= M_PI / 2 && right_angle <= M_PI;
    bool right3 = right_angle >= -M_PI && right_angle <= -M_PI/2;
    bool condition_left1_right2 = (left1) && (right2);
    bool condition_left4_right3 = (left4) && (right3);

//    std::cerr << "left_angle >= -M_PI/2: " << (left_angle >= -M_PI/2) << "\tleft_angle <= 0: " << (left_angle <= 0) << std::endl;

//    std::cerr << "condition_left1_right2: " << condition_left1_right2 << " | condition_left4_right3: " << condition_left4_right3 << std::endl;

    // 直线相撞判断
    double min_angle = 2; // 最小角度，单位：度
    double min_condition = M_PI * min_angle / 180;
//    std::cerr << "std::abs(left_angle + right_angle): " << 180 * std::abs(left_angle + right_angle - M_PI) / M_PI << std::endl;
    if ((std::abs(left_angle) + std::abs(right_angle) - M_PI) < min_condition){
        if ((left1 && right3) || (left4 && right2)){
            return true;
            std::cerr << "CRASH WARNING!!!" << std::endl;
        }
    }

    // 斜着相撞判断
    if ((left1 && right2) || (left4 && right3)){
        std::cerr << "CRASH WARNING!!!" << std::endl;
        return true;
    }

    return false;
}

float Context::GetDt() const {
    return dt;
}

void Context::FactoriesClassification() {

}

inline std::vector<int> get_the_one_position(int num)
{
    std::vector<int> positions; //用来记录所有1所在的位置
    int pos = 0; //记录1所在的位置
    while(num) {
        if(num & 1) {
            positions.push_back(pos); //如果当前位置为1，就记录下来
        }
        num >>= 1; //将num右移一位
        pos++; //位置加1
    }
    return positions;
}

std::map<FactoryType, std::pair<bool, bool>> Context::WarehouseStateConversion(FactoryType type, int rawInfo){
    std::vector<int> onePositions = get_the_one_position(rawInfo);
    std::map<FactoryType, std::pair<bool, bool>> res_;
    if (type == FACTORY_4) 
    {
        res_[MATERIAL_1].first = false;
        res_[MATERIAL_2].first = false;
        for(auto pos : onePositions)
        {
            res_[static_cast<FactoryType>(pos)].first = true;
        }
    }
    else if (type == FACTORY_5) 
    {
        res_[MATERIAL_1].first = false;
        res_[MATERIAL_3].first = false;
        for(auto pos : onePositions)
        {
            res_[static_cast<FactoryType>(pos)].first = true;
        }
    }
    else if (type == FACTORY_6) 
    {
        res_[MATERIAL_2].first = false;
        res_[MATERIAL_3].first = false;
        for(auto pos : onePositions)
        {
            res_[static_cast<FactoryType>(pos)].first = true;
        }
    }
    else if (type == FACTORY_7)
    {
        res_[FACTORY_4].first = false;
        res_[FACTORY_5].first = false;
        res_[FACTORY_6].first = false;
        for(auto pos : onePositions)
        {
            res_[static_cast<FactoryType>(pos)].first = true;
        }
    }
    return res_;
}


const int Context::GetFactoryTotalNum() const {
    return factoryTotalNum_;
}

const int Context::GetRobotTotalNum() const {
    return robotTotalNum_;
}

//std::map<FactoryType, std::map<FactoryType, std::vector<int>>> Context::GetGlobalFactoryTypeMap() const {
//    return globalFactoryTypeMap_;
//}

const std::map<FactoryType, std::map<FactoryType, std::vector<int>>> &Context::GetGlobalFactoryTypeMap() const {
    return globalFactoryTypeMap_;
}

const std::vector<std::vector<double>> &Context::GetInitialHistoryGraph() const {
    return initialHistoryGraph_;
}

const int Context::GetNodeTotalNum() const {
    return nodeTotalNum_;
}

// 打印任意图信息
void Context::PrintHistoryMap(const std::vector<std::vector<double>> &map, const std::string &title) {
    // 打印标题
    std::cerr << title << ": " << map.size() << "x" << map[0].size() << std::endl;

    // 打印行索引
    std::cerr << " \t";
    for (int i = 0; i < this->robotTotalNum_; ++i) {
        std::cerr << this->GetRobot(i)->GetRobotID() << "\t";
    }
    for (int i = 0; i < this->factoryTotalNum_; ++i) {
        std::cerr << this->GetFactory(i)->GetFactoryType() << "\t";
    }
    std::cerr << std::endl;

    for (int i = 0; i < map.size(); ++i) {
        // 打印列索引
        if (i <= 3){
            std::cerr << this->GetRobot(i)->GetRobotID() << "\t";
        }else{
            std::cerr << this->GetFactory(i- this->factoryIDShift_)->GetFactoryType() << "\t";
        }

        // 打印一整行
        for (int j = 0; j < map[i].size(); ++j) {
            std::cerr << map[i][j] << "\t";
        }
        std::cerr << std::endl;
    }
}

void Context::PrintMapMapVector(const std::map<FactoryType, std::map<FactoryType, std::vector<int>>> &map, const std::string &title) {
    std::cerr << title << ": " << std::endl;
    for (auto from : map){
        for (auto to : from.second) {
            std::cerr <<  "Key1: " << from.first << " Key2: " << to.first << " value: ";
            for (auto index : to.second) {
                std::cerr << index << " ";
            }
            std::cerr << std::endl;
        }
    }
}








