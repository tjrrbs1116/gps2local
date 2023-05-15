
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gps2local.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<s_gps2local::gps2local>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  //gps2local();
  return 0;
}
