#include "nav2_util/lifecycle_node.hpp"
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/nav_sat_fix.hpp>


namespace s_gps2local
{

class gps2local_node : public nav2_util::LifecycleNode
{

public:

  explicit gps2local_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());



// ~gps2local();


protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;



  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

};

}