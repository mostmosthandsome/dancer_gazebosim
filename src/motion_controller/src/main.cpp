#include "sim_env/simulation.h"
#include "rclcpp/rclcpp.hpp"
#include "Climb.h"

void paramater_server_thread_func(std::shared_ptr<dmotion::ParamNode> param_server)
{
  rclcpp::spin(param_server);
  RCLCPP_INFO(rclcpp::get_logger("parameter_thread"),"param server end");
}

void simulation_server_thread_func(std::shared_ptr<zjudancer::simulation> simulation_server)
{
  rclcpp::spin(simulation_server);
  RCLCPP_INFO(rclcpp::get_logger("simulation_thread"),"sim server end");
}



int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<dmotion::ParamNode> param_server_node = std::make_shared<dmotion::ParamNode>();
  std::thread parameter_server_thread(paramater_server_thread_func,param_server_node);
  dmotion::Climb climb_controller("BACK",angle,param_server_node->parameters,env);
  climb_controller.working();
  rclcpp::shutdown();

}