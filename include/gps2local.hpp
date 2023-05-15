#include "nav2_util/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <vector>
#include <rcl/time.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>
#include "nav_msgs/srv/set_map.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <queue>

#include "pluginlib/class_loader.hpp"


typedef struct
    {
    sensor_msgs::msg::NavSatFix::SharedPtr gps_data;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr map_data;
    }match_point;

namespace s_gps2local
{


    class gps2local : public rclcpp::Node{


        public:
            explicit gps2local(const rclcpp::NodeOptions &);


        private:

            void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
            void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
            sensor_msgs::msg::NavSatFix::SharedPtr current_gps_msg;
            void maketransform();
            double gps2local_trans[3][3];
            double gps[3][2];
            std::queue<match_point> temp;
            bool make_transform_ok = false;
            double gps_params[6];
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;

            

    };
}
