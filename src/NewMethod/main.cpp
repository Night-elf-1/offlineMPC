// MPC_System.hpp
#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <map>

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
                    state.yaw = deg2rad(90 - heading_error);
                    state.steering_angle = deg2rad(steering);
                    state.velocity = params_.target_v;

                    // 计算参考点和控制量
                    auto control = computeMPCControl(state, start, end);
                    
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
                                              const TrajectoryPoint& end) {
        // 计算参考点
        double ref_x, ref_y, ref_heading;
        std::tie(ref_x, ref_y, ref_heading) = calculateReferencePoint(state, start, end);

        // 计算误差
        double lat_error = calculateLateralError(state.x, state.y, ref_x, ref_y, ref_heading);
        double heading_error = normalizeAngle(state.yaw - ref_heading);

        // 简化的MPC控制律
        double steering_command = -0.5 * lat_error - 0.5 * heading_error;
        steering_command = std::clamp(steering_command, -params_.max_steer, params_.max_steer);

        return {steering_command, params_.target_v};
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

// 使用示例
int main() {
    OfflineMPCTable mpc_table;
    
    // 生成离线表
    TrajectoryPoint start{0, 0, 0};
    TrajectoryPoint end{0, 10, M_PI/2}; // 终点在(0,10)，航向角90度
    mpc_table.generateTable(start, end);

    // 使用离线表进行查询
    double lateral_error = 0.5;    // 当前横向误差
    double heading_error = 5.0;    // 当前航向角误差(度)
    double current_steering = 0.0; // 当前转向角(度)

    auto control = mpc_table.queryControl(lateral_error, heading_error, current_steering);
    std::cout << "Steering command: " << control.first << " rad\n";
    std::cout << "Velocity command: " << control.second << " m/s\n";

    return 0;
}