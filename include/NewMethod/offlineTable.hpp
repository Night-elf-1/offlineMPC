// MPC_System.hpp
#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Core"

struct parameters
{
    double L = 3.4;                             //轴距
    int NX = 3, NU = 2, NP = 60, NC = 5;        //状态变量维度，控制变量维度，预测步数，控制步长
    double dt = 0.5;                           //时间步长
    double row = 10;                            // 松弛因子  
    Eigen::VectorXd U = Eigen::VectorXd::Constant(NU, 0.01);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(NU, NU);
    Eigen::MatrixXd RB = Eigen::MatrixXd::Identity(NC*NU, NC*NU);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(NX, NX);
    Eigen::MatrixXd QB = 100*Eigen::MatrixXd::Identity(NP*NX, NP*NX);
};


struct VehicleState {
    double x;
    double y;
    double yaw;
    double steering_angle;
    double velocity;
};

struct VehicleParams {
    const double dt = 0.1;          // 时间步长
    const double L = 3.4;           // 轴距
    const double max_steer = 0.1570796; // 最大方向盘转角(rad)
    const double target_v = 0.75;    // 目标速度
};

struct TrajectoryPoint {
    double x;
    double y;
    double heading;
};

class OfflineMPCTable {
public:
    OfflineMPCTable() : params_() {}

    // 生成离线MPC表
    void generateTable(const TrajectoryPoint& start, const TrajectoryPoint& end) {
        const double lateral_error_step = 0.01;
        const double heading_error_step = 0.1;
        const double steering_angle_step = 0.1;
        // Eigen::VectorXd U = Eigen::VectorXd::Constant(parameters::NU, 0.01);

        // 遍历横向误差范围 (-1m到1m)
        for (double lateral_error = -1.0; lateral_error <= 1.0; lateral_error += lateral_error_step) {
            std::map<std::pair<double, double>, std::pair<double, double>> error_control_map;

            // 遍历航向角误差范围 (-10度到10度)
            for (double heading_error = -10.0; heading_error <= 10.0; heading_error += heading_error_step) {
                // 遍历前轮转角范围 (-9度到8.6度)
                for (double steering = -9.0; steering <= 8.6; steering += steering_angle_step) {
                    // 计算初始状态
                    VehicleState state;
                    state.x = lateral_error;
                    state.y = 0;
                    state.yaw = deg2rad(90 - heading_error);                // 相对与于参考线的航向角
                    state.steering_angle = deg2rad(steering);
                    state.velocity = params_.target_v;
                    // U(1) = deg2rad(steering);

                    // 计算参考点和控制量
                    auto control = computeMPCControl(state, start, end, deg2rad(steering));
                    
                    // 存储结果到map中
                    error_control_map[{lateral_error, heading_error}] = {control.first, control.second};
                }
            }
            
            // 保存当前横向误差下的控制映射
            saveErrorControlMap(lateral_error, error_control_map);
        }
    }

    // 查询控制量
    std::pair<double, double> queryControl(double lateral_error, double heading_error, double current_steering) {
        // 根据误差查找最近的控制量
        auto nearest_lateral = roundToStep(lateral_error, 0.01);
        auto nearest_heading = roundToStep(heading_error, 0.1);
        
        // 从保存的表中查找对应的控制量
        std::string filename = generateFilename(nearest_lateral);
        auto control = loadControl(filename, nearest_heading, current_steering);
        
        return control;
    }

private:
    VehicleParams params_;

    // MPC控制器计算
    std::pair<double, double> computeMPCControl(const VehicleState& state, 
                                              const TrajectoryPoint& start, 
                                              const TrajectoryPoint& end,
                                              const parameters& params_,
                                              Eigen::Vector3d& inital_x,
                                              const double& steering) {
        
        const double row = 10;
        Eigen::Vector2d u_min(-0.001,-0.1570796);
        Eigen::Vector2d u_max(0.001,0.1570796);
        Eigen::Vector2d delta_umin(-0.005, -0.0174533);
        Eigen::Vector2d delta_umax(0.005, 0.0174533);
        inital_x << state.x, state.y, state.yaw;
        Eigen::VectorXd U = params_.U;
        U(1) = steering;

        double yaw_r = end.heading;                                 // 航向角        
        double v_r = state.velocity;                                // 速度
        double lat_error = state.x;                                 // 横向误差
        double delta_f_r = 0.0;                                     // 参考转向角
      
        Eigen::Matrix3d Ad(3, 3);             // Ad矩阵
        Ad << 1, 0, (-1*v_r) * sin(yaw_r) * params_.dt,
            0, 1, v_r * cos(yaw_r) * params_.dt,
            0, 0, 1;

        Eigen::MatrixXd Bd(3, 2);       // Bd矩阵
        Bd << cos(yaw_r) * params_.dt, 0,
            sin(yaw_r) * params_.dt, 0,
            (tan(delta_f_r) / params_.L) * params_.dt, (v_r / (params_.L * cos(delta_f_r)*cos(delta_f_r))) * params_.dt;

        // 状态空间方程的相关矩阵
        Eigen::VectorXd kesi(params_.NX + params_.NU);             // 新状态变量kesi  3 + 2
        Eigen::Vector3d x_r(0.0, end.y, yaw_r);
        kesi.head(params_.NX) = inital_x - x_r;
        kesi.tail(params_.NU) = U;                                 // U为初始控制量

        Eigen::MatrixXd A_3 = Eigen::MatrixXd::Zero(params_.NX + params_.NU, params_.NX + params_.NU);              // A矩阵为A3矩阵
        A_3.topLeftCorner(params_.NX, params_.NX) = Ad;
        A_3.topRightCorner(params_.NX, params_.NU) = Bd;
        A_3.bottomRightCorner(params_.NU, params_.NU) = Eigen::MatrixXd::Identity(params_.NU, params_.NU);       // 往A3矩阵中添加值

        Eigen::MatrixXd B_3 = Eigen::MatrixXd::Zero(params_.NX + params_.NU, params_.NU);                       // B为B3矩阵
        B_3.topLeftCorner(params_.NX, params_.NU) = Bd;                                    // 往B3矩阵里面添加Bd矩阵和I矩阵(单位矩阵)
        B_3.bottomRightCorner(params_.NU, params_.NU) = Eigen::MatrixXd::Identity(params_.NU, params_.NU);

        // C 矩阵
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(params_.NX, params_.NX + params_.NU);               // 设置C矩阵
        C.topLeftCorner(params_.NX, params_.NX) = Eigen::MatrixXd::Identity(params_.NX, params_.NX);          // C=[E 0]

        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(params_.NP * params_.NX, params_.NX + params_.NU);       // PHI为W矩阵 
        for (int i = 0; i < params_.NP; ++i) {
            Eigen::MatrixXd result = A_3;
            for (int j = 0; j < i; ++j)
            {
                result = result * A_3;
                // count += 1;
            }
            
            W.middleRows(i * params_.NX, params_.NX) = C * result;
        }

        Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(params_.NP * params_.NX, params_.NC * params_.NU);          // THETA矩阵为Z矩阵
        for (int i = 0; i < params_.NP; ++i) {
            for (int j = 0; j < params_.NC; ++j) {
                if (j <= i) {
                    // 计算 A_3^(i-j)
                    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A_3.rows(), A_3.cols()); // 初始化为单位矩阵
                    for (int k = 0; k < (i - j); ++k) {
                        result *= A_3; // 逐步累乘A_3
                    }

                    Z.middleRows(i * params_.NX, params_.NX).middleCols(j * params_.NU, params_.NU) = C * result * B_3;
                }
            }
        }

        // H 矩阵
        Eigen::MatrixXd H = Z.transpose() * params_.QB * Z + params_.RB;
        Eigen::VectorXd g = kesi.transpose() * W.transpose() * params_.QB * Z;

        // 约束
        Eigen::MatrixXd A_e = Eigen::MatrixXd::Zero(params_.NC * params_.NU, params_.NC * params_.NU);            // A_I对应Ae矩阵
        for (int i = 0; i < params_.NC; ++i) {
            for (int j = 0; j <= i; ++j) {
                A_e.block(i * params_.NU, j * params_.NU, params_.NU, params_.NU) = Eigen::MatrixXd::Identity(params_.NU, params_.NU);
            }
        }

        Eigen::VectorXd U_t = Eigen::VectorXd::Zero(params_.NC * params_.NU);
        for (int i = 0; i < params_.NC; ++i) {
            U_t.segment(i * params_.NU, params_.NU) = U; // 复制 U 到 U_t 的每个区段
        }

        Eigen::VectorXd Umax = Eigen::VectorXd::Ones(params_.NC * params_.NU);
        for (int i = 0; i < params_.NC; ++i) {
            Umax.segment(i * params_.NU, params_.NU) = u_max;
        }

        Eigen::VectorXd Umin = Eigen::VectorXd::Ones(params_.NC * params_.NU);
        for (int i = 0; i < params_.NC; ++i) {
            Umin.segment(i * params_.NU, params_.NU) = u_min;
        }

        Eigen::VectorXd delta_Umin = Eigen::VectorXd::Ones(params_.NC * params_.NU);
        for (int i = 0; i < params_.NC; ++i) {
            delta_Umin.segment(i * params_.NU, params_.NU) = delta_umin;
        }

        Eigen::VectorXd delta_Umax = Eigen::VectorXd::Ones(params_.NC * params_.NU);
        for (int i = 0; i < params_.NC; ++i) {
            delta_Umax.segment(i * params_.NU, params_.NU) = delta_umax;
        }

        OsqpEigen::Solver solver;
        int num_variables = H.rows();
        int num_constraints = A_e.rows();

        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(num_variables);
        solver.data()->setNumberOfConstraints(num_constraints);

        // 设置二次规划问题的矩阵和向量
        Eigen::SparseMatrix<double> H_sparse = H.sparseView();
        solver.data()->setHessianMatrix(H_sparse);  // H为稀疏矩阵
        solver.data()->setGradient(g);                   // 线性项 g

        // 设置约束的矩阵和边界
        Eigen::SparseMatrix<double> A_e_sparse = A_e.sparseView();
        solver.data()->setLinearConstraintsMatrix(A_e_sparse);  // A为稀疏矩阵
        solver.data()->setLowerBound(delta_Umin);                    // 下界
        solver.data()->setUpperBound(delta_Umax);                    // 上界

        // 初始化并求解问题
        if (!solver.initSolver()) {
            throw std::runtime_error("Solver initialization failed");
        }

        solver.solveProblem();

        // 获取求解结果
        Eigen::VectorXd solution = solver.getSolution();
        // 更新控制量U
        Eigen::VectorXd delta_U = solution.head(U.size());
        //std::cout << "delta_U = " << delta_U << std::endl;
        U += delta_U;
        //std::cout << "U = " << U << std::endl;

        // 计算实际的控制量
        // double v_real = U(0) + v_r + kesi(3);
        double v_real = U(0) + v_r;
        //double delta_real = U(1) + delta_f_r + kesi(4);
        double delta_real = U(1) + delta_f_r;
        // std::cout << "参考转角 = " << delta_f_r << std::endl;
        // std::cout << "计算转角 = " << delta_real << std::endl;
        // cout << "v_real = " << v_real
        //     << "delta_real = " << delta_real << endl;

        return std::make_tuple(v_real, delta_real);

    }

    // 计算参考点
    std::tuple<double, double, double> calculateReferencePoint(const VehicleState& state,
                                                             const TrajectoryPoint& start,
                                                             const TrajectoryPoint& end) {
        // 计算轨迹方向
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double trajectory_heading = std::atan2(dy, dx);

        // 计算投影点
        double proj_x = start.x + (state.x - start.x) * std::cos(trajectory_heading) 
                               + (state.y - start.y) * std::sin(trajectory_heading);
        double proj_y = start.y - (state.x - start.x) * std::sin(trajectory_heading) 
                               + (state.y - start.y) * std::cos(trajectory_heading);

        return {proj_x, proj_y, trajectory_heading};
    }

    // 计算横向误差
    double calculateLateralError(double x, double y, double ref_x, double ref_y, double ref_heading) {
        return (y - ref_y) * std::cos(ref_heading) - (x - ref_x) * std::sin(ref_heading);
    }

    // 角度归一化
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // 弧度转度
    double rad2deg(double rad) {
        return rad * 180.0 / M_PI;
    }

    // 度转弧度
    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    // 四舍五入到最近的步长
    double roundToStep(double value, double step) {
        return std::round(value / step) * step;
    }

    // 生成文件名
    std::string generateFilename(double lateral_error) {
        return "lateral_error_" + std::to_string(int(lateral_error * 100));
    }

    // 保存误差控制映射
    void saveErrorControlMap(double lateral_error, 
                           const std::map<std::pair<double, double>, 
                           std::pair<double, double>>& error_control_map) {
        std::string filename = generateFilename(lateral_error);
        std::ofstream file(filename + ".bin", std::ios::binary);
        
        // 写入map大小
        size_t size = error_control_map.size();
        file.write(reinterpret_cast<const char*>(&size), sizeof(size));

        // 写入map内容
        for (const auto& entry : error_control_map) {
            file.write(reinterpret_cast<const char*>(&entry.first.first), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.first.second), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.second.first), sizeof(double));
            file.write(reinterpret_cast<const char*>(&entry.second.second), sizeof(double));
        }
    }

    // 加载控制量
    std::pair<double, double> loadControl(const std::string& filename, 
                                        double heading_error,
                                        double current_steering) {
        std::ifstream file(filename + ".bin", std::ios::binary);
        if (!file.is_open()) {
            return {0.0, params_.target_v}; // 默认值
        }

        size_t size;
        file.read(reinterpret_cast<char*>(&size), sizeof(size));

        // 查找最近的控制量
        double min_distance = std::numeric_limits<double>::max();
        std::pair<double, double> best_control;

        for (size_t i = 0; i < size; ++i) {
            double stored_lateral_error, stored_heading_error;
            double stored_steering, stored_velocity;

            file.read(reinterpret_cast<char*>(&stored_lateral_error), sizeof(double));
            file.read(reinterpret_cast<char*>(&stored_heading_error), sizeof(double));
            file.read(reinterpret_cast<char*>(&stored_steering), sizeof(double));
            file.read(reinterpret_cast<char*>(&stored_velocity), sizeof(double));

            double distance = std::pow(stored_heading_error - heading_error, 2) + 
                            std::pow(stored_steering - current_steering, 2);

            if (distance < min_distance) {
                min_distance = distance;
                best_control = {stored_steering, stored_velocity};
            }
        }

        return best_control;
    }
};