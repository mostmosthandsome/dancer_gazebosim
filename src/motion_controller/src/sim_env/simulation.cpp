#include "sim_env/simulation.h"
#include <thread>
using namespace std::chrono_literals;

namespace zjudancer
{
    simulation::simulation():Node("simulation_communication_node")
    {
        for(int i = 0; i < 16; ++i) pub[i] = this->create_publisher<std_msgs::msg::Float64>("/rotor" + std::to_string(i),10);

    }

    void simulation::step(std::vector<double> joint_angle)
    {
        for(int i = 0; i < 16; ++i)
        {
            auto msg = std_msgs::msg::Float64();
            msg.data = joint_angle[i] / 180 * 3.141592654;
            pub[i]->publish(msg);
        }
        std::this_thread::sleep_for(9ms);
    }

    std::vector<double> simulation::get_joint_angle()
    {
        return joint_reader.get_joint_angle();
    }
}

