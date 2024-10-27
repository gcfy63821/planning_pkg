//success
#include <franka/robot.h>
#include <franka/model.h>
#include <franka_hw/franka_model_interface.h>
#include <exception>
#include <franka/exception.h>

int main() {
    try {
        // 创建 Franka 机器人对象
        franka::Robot robot("172.16.0.2");

        // 获取当前关节位置命令
        franka::RobotState robot_state = robot.readOnce();
        franka::JointPositions joint_positions_command = robot_state.q_d;

        // 在这里使用 joint_positions_command 进行其他操作
        // 输出关节坐标信息
        std::array<double, 7> initial_position = robot_state.q_d;

        std::cout << "Current Joint Positions:" << std::endl;
        for (size_t i = 0; i < initial_position.size(); ++i) {
            std::cout << "Joint " << i + 1 << ": " << std::setprecision(4) << initial_position[i] << " radians" << std::endl;
        }


    } catch (std::exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
