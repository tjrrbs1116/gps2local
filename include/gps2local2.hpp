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
            void calTheta();
            void convertUnitToLat();
            void convertUnitToLon();
            double gps[3][2];    
            double map_data[3][2];
            std::queue<match_point> temp;
            double targetLon = 37.3;
            double radiusOfEarth = 6371.009;
            double circumferenceOfEarth = 2 * Math.PI * radiusOfEarth; //지구 둘레
            double distancePerLat = circumferenceOfEarth / 360; //경도당 거리(km)
            double distancePerLon = Math.cos(targetLon * Math.PI / 180) * circumferenceOfEarth / 360; //위도당 거리(km)

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;

            

    };
}
