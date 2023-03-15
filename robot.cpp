//
// Created by orw on 3/14/23.
//

#include "robot.h"

void setRobotRun(int robotID, int workbenchID) {
    double robotX = frameData.robotState[robotID].x;
    double robotY = frameData.robotState[robotID].y;
    double workbenchX = mapData.workbenchIDCoordinateMap[workbenchID].first;
    double workbenchY = mapData.workbenchIDCoordinateMap[workbenchID].second;
    double orientationNeed = 0;
    double orientationBias = 0;

    // 转动
    if (workbenchX == robotX) {  // 在一条竖直的直线上
        if (workbenchY > robotY) {  // 需要往上
            orientationNeed = pi / 2;
            if (frameData.robotState[robotID].orientation > 0) {  // 目前朝上
                if (frameData.robotState[robotID].orientation > orientationNeed) {  // 偏左
                    printf("rotate %d %f\n", robotID, -angleSpeed);  // 顺时针为负
                } else {  // 偏右
                    printf("rotate %d %f\n", robotID, angleSpeed);  // 逆时针为正
                }
            } else {  // 目前朝下
                if (-frameData.robotState[robotID].orientation > orientationNeed) {  // 偏左
                    printf("rotate %d %f\n", robotID, -angleSpeed);  // 顺时针为负
                } else {  // 偏右
                    printf("rotate %d %f\n", robotID, angleSpeed);  // 逆时针为正
                }
            }
        } else {  // 需要往下
            orientationNeed = -pi / 2;
            if (frameData.robotState[robotID].orientation < 0) {  // 目前朝下
                if (frameData.robotState[robotID].orientation < orientationNeed) {  // 偏左
                    printf("rotate %d %f\n", robotID, -angleSpeed);  // 顺时针为负
                } else {  // 偏右
                    printf("rotate %d %f\n", robotID, angleSpeed);  // 逆时针为正
                }
            } else {  // 目前朝上
                if (-frameData.robotState[robotID].orientation < orientationNeed) {  // 偏左
                    printf("rotate %d %f\n", robotID, -angleSpeed);  // 顺时针为负
                } else {  // 偏右
                    printf("rotate %d %f\n", robotID, angleSpeed);  // 逆时针为正
                }
            }
        }
    } else {  // 正常情况，不在一条竖直的直线上
        orientationNeed = atan2(workbenchY - robotY, workbenchX - robotX);  // 机器人去往工作台需要的朝向
        orientationBias = orientationNeed - frameData.robotState[robotID].orientation;  // 所需朝向减去机器人当前朝向
        if (orientationNeed >= 0) {  // 工作台在机器人上面
            if (orientationBias >= 0 && orientationBias <= pi) {  // 说明机器人在其右半边扇区，需要从上面往左转
                printf("rotate %d %f\n", robotID, angleSpeed);  // 逆时针
            } else {
                printf("rotate %d %f\n", robotID, -angleSpeed);  // 顺时针
            }
        } else {  // 工作台在机器人下面
            if (orientationBias <= 0 && orientationBias >= -pi) {  // 说明机器人在其右半边扇区，需要从下面往左转
                printf("rotate %d %f\n", robotID, -angleSpeed);  // 顺时针
            } else {
                printf("rotate %d %f\n", robotID, angleSpeed);  // 逆时针
            }
        }
    }

    // 平动
    if (fabs(orientationBias) > pi / 4) {  // 角度偏差太大，先不要转动
        printf("forward %d %d\n", robotID, 0);
    } else {
        printf("forward %d %d\n", robotID, 6);
    }
}