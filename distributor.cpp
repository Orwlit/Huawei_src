//
// Created by orw on 3/14/23.
//
#include <iostream>
#include <cmath>

#include "distributor.h"


Distributor::Distributor() {
    std::shared_ptr<Context> sharedPtr = std::make_shared<Context>();
    this->context = sharedPtr;
}

bool Distributor::run() {
    while (this->context->UpdateAllStatus()){

        //开始与控制台交互
        std::cout << this->context->GetFrameId() << std::endl;

//        std::cout << "forward 0 6" << std::endl;
//        std::cout << "rotate 0 " << M_PI << std::endl;


        this->context->GetRobot(0)->HighSpeedMove(40.0f, 20.0f, this->context->GetDt());
        this->context->GetRobot(3)->HighSpeedMove(10.0f, 20.0f, this->context->GetDt());

//        if (this->context->GetRobot(0)->GetLinearVelocity() < 0.1){
//            std::cerr << "Robot 0 STOPPED!!!" << std::endl;
//        }
//        if (this->context->GetRobot(1)->GetLinearVelocity() < 0.1){
//            std::cerr << "Robot 1 STOPPED!!!" << std::endl;
//        }
//        this->context->GetRobot(2)->HighSpeedMove(25.0f, 25.0f, this->context->GetDt());
//        this->context->GetRobot(3)->HighSpeedMove(25.0f, 25.0f, this->context->GetDt());


//        if (!(this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(1), 8.5))){
//            std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
//        }

        std::cerr << "Frame ID: " << this->context->GetFrameId() << std::endl;
        bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(0), this->context->GetRobot(3), 22);
        if (aboutToCrash){
            this->context->GetRobot(0)->Rotate(M_PI/2);
            this->context->GetRobot(3)->Rotate(M_PI/2);
        }
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                bool aboutToCrash = this->context->AboutToCrash(this->context->GetRobot(i), this->context->GetRobot(j), 9);
//                if (aboutToCrash){
//                    this->context->GetRobot(i)->Forward(0);
//                    this->context->GetRobot(j)->Forward(0);
//                }
//            }
//        }

        //与控制台交互结束
        std::cout << "OK" << std::flush;

        this->context->SetPreviousFrameId(this->context->GetFrameId());
    }
    return false;
}

bool Distributor::StrategyGlobal() {

    return false;
}

bool Distributor::MaintainGraph() {



    return false;
}

void Distributor::FloydMinDistance() const {

}

bool Distributor::GraphOptimization() {


    return false;
}



