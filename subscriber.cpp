//
// Created by orw on 3/14/23.
//
#include <map>
#include <vector>
#include <mutex>

#include "robot.h"
#include "factory.h"

//用作更新机器人和工厂的信息，单例模式
class Subscriber{
private:
    static Subscriber* instance; // 单例实例指针
    static std::mutex mtx; // 互斥锁

    std::map<int, std::pair<int, int>> factory_coordinate;
    std::vector<int> producing_time;
public:
    Subscriber();
    static Subscriber* getInstance() { // 获取单例实例的静态方法
        if (instance == nullptr) {
            std::lock_guard<std::mutex> lock(mtx); // 加锁
            if (instance == nullptr) { // 双重检查锁
                instance = new Subscriber(); // 如果实例不存在，创建一个新的实例
            }
        }
        return instance; // 返回单例实例指针
    }

    bool UpdateRobot(Robot* robot){

    };

    bool UpdateFactory(Factory* factory){

    };

};