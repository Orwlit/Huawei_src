//
// Created by orw on 3/14/23.
//

#include "utilities.h"

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

// 机器人动态系统的状态向量结构体
struct State {
    double x;   // 机器人的x坐标
    double y;   // 机器人的y坐标
    double theta;   // 机器人的朝向角度（弧度制）
};

// 定义机器人的控制输入向量结构体
struct Control {
    double v;   // 机器人的线速度
    double w;   // 机器人的角速度
};

// 定义Pontryagin最小时间问题的类
class PontryaginMinimumTime {
public:
    PontryaginMinimumTime() {
        // 设置机器人的约束条件
        max_v_ = 1.0;
        max_w_ = 0.5;
    }

    // 定义机器人的动态系统函数
    State f(State x, Control u) {
        State xdot;
        xdot.x = x.x + u.v * cos(x.theta);
        xdot.y = x.y + u.v * sin(x.theta);
        xdot.theta = x.theta + u.w;
        return xdot;
    }

    // 定义Hamilton函数
    double H(State x, Control u, vector<double> lambda) {
        double h = 0.0;
        h += lambda[0] * u.v;
        h += lambda[1] * u.w;
        return h;
    }

    // 定义Hamilton-Jacobi-Bellman(HJB)方程
    double HJB(State x, double t, vector<double> lambda) {
        double min_h = INFINITY;
        for (double v = -max_v_; v <= max_v_; v += 0.01) {
            for (double w = -max_w_; w <= max_w_; w += 0.01) {
                Control u;
                u.v = v;
                u.w = w;
                double h = H(x, u, lambda);
                State xdot = f(x, u);
                double HJB_val = -h + dot(lambda, xdot);
                if (HJB_val < min_h) {
                    min_h = HJB_val;
                }
            }
        }
        return min_h;
    }

    // 定义点积函数
    double dot(vector<double> v1, State x) {
        double dot_val = 0.0;
        dot_val += v1[0] * cos(x.theta);
        dot_val += v1[1] * sin(x.theta);
        dot_val += v1[2];
        return dot_val;
    }

    // 定义Pontryagin最小时间问题的求解函数
    double solve(State x0, State x1) {
        double dt = 0.1;   // 时间间隔
        double t_max = 10.0;   // 最大时间
        int N = t_max / dt;   // 离散时间步数
        vector<State> x(N+1);   // 机
        x[0] = x0;
        vector<double> lambda(3, 0.0);   // 初始lambda向量为0
        for (int i = 0; i < N; i++) {
            // 使用Euler方法进行数值积分
            State xdot = f(x[i], control[i]);
            x[i+1].x = x[i].x + xdot.x * dt;
            x[i+1].y = x[i].y + xdot.y * dt;
            x[i+1].theta = x[i].theta + xdot.theta * dt;

            // 计算HJB方程
            double HJB_val = HJB(x[i+1], (N-i-1)*dt, lambda);

            // 计算lambda的变化率
            vector<double> lambda_dot(3, 0.0);
            for (double v = -max_v_; v <= max_v_; v += 0.01) {
                for (double w = -max_w_; w <= max_w_; w += 0.01) {
                    Control u;
                    u.v = v;
                    u.w = w;
                    double h = H(x[i+1], u, lambda);
                    State xdot = f(x[i+1], u);
                    double dot_val = dot(lambda, xdot);
                    double lambda_dot_val = -h - HJB_val * dot_val;
                    lambda_dot[0] += lambda[1] * lambda_dot_val;
                    lambda_dot[1] += lambda[2] * lambda_dot_val;
                    lambda_dot[2] += -v * lambda_dot_val * sin(x[i+1].theta)
                                     + w * lambda_dot_val * cos(x[i+1].theta);
                }
            }

            // 使用Euler方法计算lambda的变化
            for (int j = 0; j < 3; j++) {
                lambda[j] += lambda_dot[j] * dt;
            }
        }

        // 计算最小时间
        double min_time = 0.0;
        for (int i = 0; i < N; i++) {
            double dx = x[i+1].x - x[i].x;
            double dy = x[i+1].y - x[i].y;
            double ds = sqrt(dx*dx + dy*dy);
            min_time += ds / control[i].v;
        }

        return min_time;
    }
private:
    double max_v_; // 机器人的最大线速度
    double max_w_; // 机器人的最大角速度
    vector<Control> control; // 机器人的控制输入向量
};

//int main() {
//    PontryaginMinimumTime pmt;
//    State x0, x1;
//    x0.x = 0.0;
//    x0.y = 0.0;
//    x0.theta = 0.0;
//    x1.x = 1.0;
//    x1.y = 1.0;
//    x1.theta = M_PI / 2.0;
//    double min_time = pmt.solve(x0, x1);
//    cout << "Minimum time: " << min_time << endl;
//    return 0;
//}