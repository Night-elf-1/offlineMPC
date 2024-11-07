#include "MpcLookupTable.hpp"

int main() {
    // 1. 首先生成离线表（只需要执行一次）
    MpcLookupTable table(
        -2.0, 2.0, 0.1,     // 横向误差范围和步长
        -M_PI/4, M_PI/4, 0.1,  // 航向角误差范围和步长
        -M_PI/6, M_PI/6, 0.1   // 转角误差范围和步长
    );
    
    table.generateTable();
    table.saveTable("../data/lookup_table.bin");
    
    // 2. 实际使用时，只需要加载离线表
    MpcLookupTable runtime_table(-2.0, 2.0, 0.1,
                                -M_PI/4, M_PI/4, 0.1,
                                -M_PI/6, M_PI/6, 0.1);
    runtime_table.loadTable("../data/lookup_table.bin");
    
    // 3. 运行时使用
    // 可以处理任意终点的直线跟踪任务
    State current_state = {0.0, 0.0, 0.0, 0.0};  // 当前状态
    TargetLine target1 = {10.0, 0.0, 0.0};       // 任务1：直线终点
    TargetLine target2 = {5.0, 5.0, M_PI/4};     // 任务2：直线终点
    
    // 查询最优控制输入
    double steering1 = runtime_table.queryOptimalSteering(current_state, target1);
    double steering2 = runtime_table.queryOptimalSteering(current_state, target2);
}