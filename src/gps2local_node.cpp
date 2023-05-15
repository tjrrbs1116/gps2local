
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "gps2local_node.hpp"
#include "gps2local.hpp"


namespace s_gps2local
{

gps2local_node::gps2local_node(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("gps2loal","",options)

{

RCLCPP_INFO(get_logger(), "gps2local creating");
}


nav2_util::CallbackReturn
gps2local_node::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

 
  return nav2_util::CallbackReturn::SUCCESS;

}



nav2_util::CallbackReturn
gps2local_node::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "this is activate");
  const rclcpp::NodeOptions options;
  auto node2 = std::make_shared<gps2local>(options);

  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
gps2local_node::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn
gps2local_node::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

}
nav2_util::CallbackReturn
gps2local_node::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

} //namespace