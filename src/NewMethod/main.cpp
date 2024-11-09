#include "../../include/NewMethod/offlineTable.hpp"

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