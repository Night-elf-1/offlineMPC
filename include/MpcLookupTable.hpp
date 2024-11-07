// #pragma once
// #include <vector>
// #include <string>
// #include <cmath>
// #include <fstream>

// struct State {
//     double x;        // x坐标
//     double y;        // y坐标
//     double theta;    // 航向角
//     double delta;    // 前轮转角
// };

// struct Error {
//     double lateral;  // 横向误差
//     double heading;  // 航向角误差
//     double steering; // 转角误差
// };

// class MpcLookupTable {
// public:
//     MpcLookupTable(
//         double lateral_min, double lateral_max, double lateral_step,
//         double heading_min, double heading_max, double heading_step,
//         double steering_min, double steering_max, double steering_step
//     );

//     // 生成离线表
//     void generateTable(double goal_x, double goal_y, double goal_theta);
    
//     // 保存离线表到文件
//     void saveTable(const std::string& filename) const;
    
//     // 从文件加载离线表
//     void loadTable(const std::string& filename);
    
//     // 在线查询最优控制输入
//     double queryOptimalSteering(const State& current_state) const;
    
// private:
//     // 计算误差
//     Error calculateErrors(const State& current_state) const;
    
//     // 计算点到直线的距离
//     double calculateLateralError(double x, double y) const;
    
//     // MPC求解器(这里简化为基于误差的PID控制)
//     double solveMPC(const Error& errors) const;
    
//     // 三线性插值
//     double trilinearInterpolation(const Error& errors) const;
    
//     // 表格参数
//     std::vector<double> lateral_grid;   // 横向误差网格
//     std::vector<double> heading_grid;   // 航向角误差网格
//     std::vector<double> steering_grid;  // 转角误差网格
    
//     // 存储查找表数据
//     std::vector<std::vector<std::vector<double>>> lookup_table;
    
//     // 目标直线参数
//     double target_x, target_y, target_theta;
//     double line_a, line_b, line_c;  // ax + by + c = 0 直线方程参数
// };



#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

struct State {
    double x;        // x坐标
    double y;        // y坐标
    double theta;    // 航向角
    double delta;    // 前轮转角
};

struct Error {
    double lateral;  // 横向误差
    double heading;  // 航向角误差
    double steering; // 转角误差
};

// 目标直线信息
struct TargetLine {
    double x;      // 终点x坐标
    double y;      // 终点y坐标
    double theta;  // 终点航向角
};

class MpcLookupTable {
public:
    MpcLookupTable(
        double lateral_min, double lateral_max, double lateral_step,
        double heading_min, double heading_max, double heading_step,
        double steering_min, double steering_max, double steering_step
    );

    // 生成离线表(只需要生成一次)
    void generateTable();
    
    // 保存离线表到文件
    void saveTable(const std::string& filename) const;
    
    // 从文件加载离线表
    void loadTable(const std::string& filename);
    
    // 在线查询最优控制输入
    double queryOptimalSteering(
        const State& current_state,
        const TargetLine& target_line
    ) const;
    
private:
    // 将当前状态转换到标准化坐标系
    Error normalizeErrors(
        const State& current_state,
        const TargetLine& target_line
    ) const;
    
    // MPC求解器(这里简化为基于误差的PID控制)
    double solveMPC(const Error& errors) const;
    
    // 三线性插值
    double trilinearInterpolation(const Error& errors) const;

    // 坐标转换相关函数
    void transformToLocal(
        double global_x, double global_y, double global_theta,
        const State& reference_state,
        double& local_x, double& local_y, double& local_theta
    ) const;
    
    // 表格参数
    std::vector<double> lateral_grid;   // 横向误差网格
    std::vector<double> heading_grid;   // 航向角误差网格
    std::vector<double> steering_grid;  // 转角误差网格
    
    // 存储查找表数据
    std::vector<std::vector<std::vector<double>>> lookup_table;
};