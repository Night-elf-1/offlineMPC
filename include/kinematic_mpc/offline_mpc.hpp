#include <vector>
#include <cmath>
#include <fstream>
#include <array>
#include <algorithm>
#include <iostream>

// 定义状态和控制量的结构体
struct State {
    double x;        // x坐标
    double y;        // y坐标
    double theta;    // 航向角
    double delta;    // 前轮转角
};

struct Control {
    double steering; // 转向控制量
    double velocity; // 速度控制量
};

// 定义离线表的数据结构
struct TableEntry {
    double lateral_error;    // 横向误差
    double heading_error;    // 航向误差
    double steering_error;   // 转角误差
    double optimal_steering; // 最优转角
    double optimal_velocity; // 最优速度
};

class MPCLookupTable {
private:
    std::vector<TableEntry> lookup_table;
    const double L = 2.7;  // 轴距
    const double dt = 0.1; // 时间步长
    const int prediction_horizon = 20; // 预测时域
    
    // 离散化的误差范围
    const double lateral_error_range = 2.0;    // ±2米
    const double heading_error_range = M_PI/2; // ±90度
    const double steering_error_range = M_PI/6;// ±30度
    const int discretization_points = 20;      // 每个维度的离散化点数
    
    // 计算点到直线的距离和误差
    void calculateErrors(const State& current_state, 
                        const State& target_state,
                        double& lateral_error,
                        double& heading_error,
                        double& steering_error) {
        // 计算目标直线的方向向量
        double dx = cos(target_state.theta);
        double dy = sin(target_state.theta);
        
        // 计算横向误差（点到直线距离）
        lateral_error = (current_state.y - target_state.y) * dx - 
                       (current_state.x - target_state.x) * dy;
        
        // 计算航向误差
        heading_error = normalizeAngle(current_state.theta - target_state.theta);
        
        // 计算转角误差
        steering_error = normalizeAngle(current_state.delta - 0.0); // 假设目标转角为0
    }
    
    // 角度归一化到[-π, π]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    // 运动学模型
    State propagateModel(const State& state, const Control& control) {
        State next_state;
        next_state.x = state.x + control.velocity * cos(state.theta) * dt;
        next_state.y = state.y + control.velocity * sin(state.theta) * dt;
        next_state.theta = state.theta + control.velocity * tan(control.steering) / L * dt;
        next_state.delta = control.steering;
        return next_state;
    }
    
    // MPC优化求解（简化版）
    Control solveOptimalControl(const State& current_state,
                              const State& target_state,
                              double lateral_error,
                              double heading_error,
                              double steering_error) {
        // 这里使用简化的PID式控制律来代替完整的MPC优化
        // 实际应用中应该使用更复杂的优化求解器
        Control control;
        const double Kp_lateral = 0.5;
        const double Kp_heading = 1.0;
        const double Kp_steering = 0.3;
        
        control.steering = Kp_lateral * lateral_error +
                         Kp_heading * heading_error +
                         Kp_steering * steering_error;
        
        // 限制转角范围
        control.steering = std::max(-M_PI/6, std::min(M_PI/6, control.steering));
        
        // 设置固定速度
        control.velocity = 1.0;
        
        return control;
    }

public:
    // 生成离线表
    void generateTable() {
        for (int i = 0; i < discretization_points; ++i) {
            for (int j = 0; j < discretization_points; ++j) {
                for (int k = 0; k < discretization_points; ++k) {
                    // 计算离散化的误差值
                    double lateral = -lateral_error_range + 
                        2 * lateral_error_range * i / (discretization_points - 1);
                    double heading = -heading_error_range + 
                        2 * heading_error_range * j / (discretization_points - 1);
                    double steering = -steering_error_range + 
                        2 * steering_error_range * k / (discretization_points - 1);
                    
                    // 创建当前状态和目标状态
                    State current_state = {0, lateral, heading, steering};
                    State target_state = {0, 0, 0, 0};
                    
                    // 求解最优控制
                    Control optimal_control = solveOptimalControl(
                        current_state, target_state, lateral, heading, steering);
                    
                    // 存储表项
                    TableEntry entry = {
                        lateral,
                        heading,
                        steering,
                        optimal_control.steering,
                        optimal_control.velocity
                    };
                    lookup_table.push_back(entry);
                }
            }
        }
    }
    
    // 保存离线表到文件
    void saveTable(const std::string& filename) {
        std::ofstream file(filename, std::ios::binary);
        if (file.is_open()) {
            size_t size = lookup_table.size();
            file.write(reinterpret_cast<const char*>(&size), sizeof(size));
            file.write(reinterpret_cast<const char*>(lookup_table.data()), 
                      size * sizeof(TableEntry));
            file.close();
        }
    }
    
    // 从文件加载离线表
    void loadTable(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (file.is_open()) {
            size_t size;
            file.read(reinterpret_cast<char*>(&size), sizeof(size));
            lookup_table.resize(size);
            file.read(reinterpret_cast<char*>(lookup_table.data()), 
                     size * sizeof(TableEntry));
            file.close();
        }
    }
    
    // 查找最近的控制量
    Control lookupControl(const State& current_state, const State& target_state) {
        double lateral_error, heading_error, steering_error;
        calculateErrors(current_state, target_state, 
                       lateral_error, heading_error, steering_error);
        
        // 找到最近的表项
        double min_distance = std::numeric_limits<double>::max();
        Control result = {0, 1.0}; // 默认控制量
        
        for (const auto& entry : lookup_table) {
            double distance = 
                pow(lateral_error - entry.lateral_error, 2) +
                pow(heading_error - entry.heading_error, 2) +
                pow(steering_error - entry.steering_error, 2);
            
            if (distance < min_distance) {
                min_distance = distance;
                result.steering = entry.optimal_steering;
                result.velocity = entry.optimal_velocity;
            }
        }
        
        return result;
    }
};