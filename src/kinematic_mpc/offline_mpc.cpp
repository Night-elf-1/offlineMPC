#include "../../include/kinematic_mpc/offline_mpc.hpp"

int main() {
    MPCLookupTable mpc_table;
    
    // 生成离线表
    std::cout << "Generating lookup table..." << std::endl;
    mpc_table.generateTable();
    
    // 保存离线表
    mpc_table.saveTable("mpc_lookup_table.bin");
    
    // 加载离线表
    mpc_table.loadTable("mpc_lookup_table.bin");
    
    // 测试查找控制量
    State current_state = {0, 1, 0.1, 0};  // 当前状态
    State target_state = {10, 0, 0, 0};    // 目标状态
    
    Control control = mpc_table.lookupControl(current_state, target_state);
    
    std::cout << "Optimal steering: " << control.steering << std::endl;
    std::cout << "Optimal velocity: " << control.velocity << std::endl;
    
    return 0;
}