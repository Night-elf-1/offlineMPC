#include "MpcLookupTable.hpp"

MpcLookupTable::MpcLookupTable(
    double lateral_min, double lateral_max, double lateral_step,
    double heading_min, double heading_max, double heading_step,
    double steering_min, double steering_max, double steering_step
) {
    // 初始化网格
    for (double e = lateral_min; e <= lateral_max; e += lateral_step)
        lateral_grid.push_back(e);
    for (double h = heading_min; h <= heading_max; h += heading_step)
        heading_grid.push_back(h);
    for (double s = steering_min; s <= steering_max; s += steering_step)
        steering_grid.push_back(s);
        
    // 初始化查找表大小
    lookup_table.resize(lateral_grid.size());
    for (auto& lat : lookup_table) {
        lat.resize(heading_grid.size());
        for (auto& head : lat) {
            head.resize(steering_grid.size());
        }
    }
}

void MpcLookupTable::generateTable() {
    // 在标准化坐标系下生成查找表
    // 在标准化坐标系中，目标直线永远是从原点出发，沿x轴正方向的直线
    for (size_t i = 0; i < lateral_grid.size(); i++) {
        for (size_t j = 0; j < heading_grid.size(); j++) {
            for (size_t k = 0; k < steering_grid.size(); k++) {
                Error errors{
                    lateral_grid[i],     // 横向误差
                    heading_grid[j],     // 航向角误差
                    steering_grid[k]     // 转角误差
                };
                lookup_table[i][j][k] = solveMPC(errors);
            }
        }
    }
}

void MpcLookupTable::saveTable(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    
    // 保存网格信息
    size_t size;
    
    size = lateral_grid.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));
    file.write(reinterpret_cast<const char*>(lateral_grid.data()), 
               size * sizeof(double));
    
    size = heading_grid.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));
    file.write(reinterpret_cast<const char*>(heading_grid.data()), 
               size * sizeof(double));
    
    size = steering_grid.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(size));
    file.write(reinterpret_cast<const char*>(steering_grid.data()), 
               size * sizeof(double));
    
    // 保存查找表数据
    for (const auto& lat : lookup_table) {
        for (const auto& head : lat) {
            file.write(reinterpret_cast<const char*>(head.data()), 
                      head.size() * sizeof(double));
        }
    }
}

void MpcLookupTable::transformToLocal(
    double global_x, double global_y, double global_theta,
    const State& reference_state,
    double& local_x, double& local_y, double& local_theta
) const {
    // 平移
    double dx = global_x - reference_state.x;
    double dy = global_y - reference_state.y;
    
    // 旋转
    double cos_theta = std::cos(-reference_state.theta);
    double sin_theta = std::sin(-reference_state.theta);
    
    local_x = dx * cos_theta - dy * sin_theta;
    local_y = dx * sin_theta + dy * cos_theta;
    local_theta = global_theta - reference_state.theta;
    
    // 标准化角度到[-π, π]
    local_theta = std::fmod(local_theta + M_PI, 2 * M_PI) - M_PI;
}

Error MpcLookupTable::normalizeErrors(
    const State& current_state,
    const TargetLine& target_line
) const {
    Error errors;
    
    // 1. 将当前状态转换到以车辆为原点的坐标系
    double local_target_x, local_target_y, local_target_theta;
    transformToLocal(
        target_line.x, target_line.y, target_line.theta,
        current_state,
        local_target_x, local_target_y, local_target_theta
    );
    
    // 2. 计算标准化后的误差
    // 在局部坐标系中，目标直线的方向就是local_target_theta
    double line_angle = std::atan2(local_target_y, local_target_x);
    
    // 横向误差：点到直线的距离
    errors.lateral = -local_target_y * std::cos(line_angle) + 
                     local_target_x * std::sin(line_angle);
    
    // 航向角误差：当前航向与目标直线的夹角
    errors.heading = local_target_theta;
    
    // 转角误差：直接使用当前转角
    errors.steering = current_state.delta;
    
    return errors;
}

double MpcLookupTable::queryOptimalSteering(
    const State& current_state,
    const TargetLine& target_line
) const {
    // 1. 将当前状态标准化到相对坐标系
    Error normalized_errors = normalizeErrors(current_state, target_line);
    
    // 2. 使用标准化后的误差查表
    return trilinearInterpolation(normalized_errors);
}

double MpcLookupTable::solveMPC(const Error& errors) const {
    // 这里使用简化的PID控制代替完整MPC
    // 实际应用中应该使用真正的MPC求解器
    const double Kp_lat = 1.0;
    const double Kp_heading = 2.0;
    const double Kp_steering = 0.5;
    
    return -(Kp_lat * errors.lateral + 
             Kp_heading * errors.heading + 
             Kp_steering * errors.steering);
}

double MpcLookupTable::trilinearInterpolation(const Error& errors) const {
    // 找到最近的网格点索引
    size_t i0 = 0, i1 = 0;
    size_t j0 = 0, j1 = 0;
    size_t k0 = 0, k1 = 0;
    
    // 查找横向误差的相邻点
    for (size_t i = 0; i < lateral_grid.size() - 1; i++) {
        if (errors.lateral >= lateral_grid[i] && 
            errors.lateral <= lateral_grid[i + 1]) {
            i0 = i;
            i1 = i + 1;
            break;
        }
    }
    
    // 查找航向角误差的相邻点
    for (size_t j = 0; j < heading_grid.size() - 1; j++) {
        if (errors.heading >= heading_grid[j] && 
            errors.heading <= heading_grid[j + 1]) {
            j0 = j;
            j1 = j + 1;
            break;
        }
    }
    
    // 查找转角误差的相邻点
    for (size_t k = 0; k < steering_grid.size() - 1; k++) {
        if (errors.steering >= steering_grid[k] && 
            errors.steering <= steering_grid[k + 1]) {
            k0 = k;
            k1 = k + 1;
            break;
        }
    }
    
    // 计算插值权重
    double xd = (errors.lateral - lateral_grid[i0]) / 
                (lateral_grid[i1] - lateral_grid[i0]);
    double yd = (errors.heading - heading_grid[j0]) / 
                (heading_grid[j1] - heading_grid[j0]);
    double zd = (errors.steering - steering_grid[k0]) / 
                (steering_grid[k1] - steering_grid[k0]);
    
    // 三线性插值
    double c00 = lookup_table[i0][j0][k0] * (1 - xd) + lookup_table[i1][j0][k0] * xd;
    double c10 = lookup_table[i0][j1][k0] * (1 - xd) + lookup_table[i1][j1][k0] * xd;
    double c01 = lookup_table[i0][j0][k1] * (1 - xd) + lookup_table[i1][j0][k1] * xd;
    double c11 = lookup_table[i0][j1][k1] * (1 - xd) + lookup_table[i1][j1][k1] * xd;
    
    double c0 = c00 * (1 - yd) + c10 * yd;
    double c1 = c01 * (1 - yd) + c11 * yd;
    
    return c0 * (1 - zd) + c1 * zd;
}



void MpcLookupTable::loadTable(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }
    
    // 加载网格信息
    size_t size;
    
    file.read(reinterpret_cast<char*>(&size), sizeof(size));
    lateral_grid.resize(size);
    file.read(reinterpret_cast<char*>(lateral_grid.data()), 
              size * sizeof(double));
    
    file.read(reinterpret_cast<char*>(&size), sizeof(size));
    heading_grid.resize(size);
    file.read(reinterpret_cast<char*>(heading_grid.data()), 
              size * sizeof(double));
    
    file.read(reinterpret_cast<char*>(&size), sizeof(size));
    steering_grid.resize(size);
    file.read(reinterpret_cast<char*>(steering_grid.data()), 
              size * sizeof(double));
    
    // 加载查找表数据
    lookup_table.resize(lateral_grid.size());
    for (auto& lat : lookup_table) {
        lat.resize(heading_grid.size());
        for (auto& head : lat) {
            head.resize(steering_grid.size());
            file.read(reinterpret_cast<char*>(head.data()), 
                     head.size() * sizeof(double));
        }
    }

    if (file.fail()) {
        throw std::runtime_error("Error reading from file: " + filename);
    }
}

// Error MpcLookupTable::calculateErrors(const State& current_state) const {
//     Error errors;
    
//     // 计算横向误差 (点到直线距离)
//     errors.lateral = calculateLateralError(current_state.x, current_state.y);
    
//     // 计算航向角误差 (当前航向角与目标航向角的差)
//     errors.heading = std::fmod(current_state.theta - target_theta + M_PI, 2 * M_PI) - M_PI;
    
//     // 计算转角误差
//     errors.steering = current_state.delta;
    
//     return errors;
// }

// double MpcLookupTable::calculateLateralError(double x, double y) const {
//     // 计算点到直线的距离
//     return (line_a * x + line_b * y + line_c) / 
//            std::sqrt(line_a * line_a + line_b * line_b);
// }