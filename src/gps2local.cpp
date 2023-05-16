
#include "gps2local.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define limit_cov 0.5 //0.5
#define gps_limit_cov 0.1
#define limit_dt 0.05


namespace s_gps2local
{


   gps2local::gps2local(const rclcpp::NodeOptions & options)
   : Node("gps_sub" , options)
        {
            RCLCPP_INFO(get_logger(), "this is gps2_sub");
            auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));
            auto subscriber_options = rclcpp::SubscriptionOptions();
            rclcpp::PublisherOptions publisher_options;
            publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

            gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "gps/fix", custom_qos, std::bind(&gps2local::gpsCallback, this, _1),
                subscriber_options);

            amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "amcl_pose", custom_qos , std::bind(&gps2local::amclCallback, this, _1),
                subscriber_options);

            position_pub_ =
                this->create_publisher<nav_msgs::msg::Odometry>(
                "gps_trans/odom", rclcpp::QoS(10), publisher_options);
        }




    void gps2local::gpsCallback(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "gps call back");
    this->current_gps_msg = msg ;
    // RCLCPP_INFO(get_logger(), "real_data IS %.15lf", msg->longitude);
    // RCLCPP_INFO(get_logger(), "real_data IS %.15lf", msg->latitude);
    if(this->make_transform_ok)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "map";
        odom.pose.pose.position.x = (gps_params[0]* (msg->longitude - gps_params[2])) + (gps_params[1] * (msg->latitude - gps_params[5]));
        odom.pose.pose.position.y = (gps_params[3]* (msg->longitude - gps_params[2])) + (gps_params[4] * (msg->latitude - gps_params[5]));
        position_pub_->publish(odom);
    }

}


    void gps2local::amclCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(), "amcl call back");
    rclcpp::Time t;
    // double seconds = msg->header.stamp.seconds();
    // rclcpp::Time t2(static_cast<uin64_t>(seconds * 1e9))l

    float dt = (msg->header.stamp.sec - current_gps_msg->header.stamp.sec ) + (msg->header.stamp.nanosec/1e9 - current_gps_msg->header.stamp.nanosec/1e9);
    float x_cov = msg->pose.covariance[0];
    float y_cov = msg->pose.covariance[7];


    if( x_cov+y_cov < limit_cov ){
        float gps_x_cov =this->current_gps_msg->position_covariance[0];
        float gps_y_cov =this->current_gps_msg->position_covariance[4];
        uint8_t g_status = this->current_gps_msg->status.status;

        float distance = sqrtf(pow((float)msg->pose.pose.position.x - this->previous_x ,2) + pow((float)msg->pose.pose.position.y - this->previous_y,2));
        RCLCPP_INFO(get_logger(), "distance is %f ",distance);
        RCLCPP_INFO(get_logger(), "dt is %f ",dt);
        if(g_status==0 && gps_x_cov+gps_y_cov < gps_limit_cov && fabs(dt)< limit_dt && distance >3.5)
        {

        if(temp.size()<3 && !this->make_transform_ok){

        // RCLCPP_INFO(get_logger(), "status is %d",g_status);
        // RCLCPP_INFO(get_logger(), "covariance  is %f , %f",gps_x_cov,gps_y_cov);
        // RCLCPP_INFO(get_logger(), "amcl call back time is %f ",dt);
        match_point temp_point;
        temp_point.gps_data = this->current_gps_msg;
        temp_point.map_data = msg;
        RCLCPP_INFO(get_logger(), "mapdata x is %.15lf ",msg->pose.pose.position.x);
        RCLCPP_INFO(get_logger(), "mapdata y is %.15lf ",msg->pose.pose.position.y);
        this->previous_x = (float)msg->pose.pose.position.x ;
        this->previous_y = (float)msg->pose.pose.position.y ;
        this->temp.push(temp_point);
        if(temp.size() ==3){maketransform();}}


        }

    }

}
    void gps2local::maketransform(){
        Eigen::MatrixXd wow(3,3);
        wow.setZero();
        for(int i=0; i<3; i++){
            match_point temps_= this->temp.front();
            for(int j=0; j<3; j++)
            {
                if (j ==0){wow(i,j) = temps_.map_data->pose.pose.position.x; gps[i][j] = temps_.gps_data->longitude; }
                if (j ==1){wow(i,j) = temps_.map_data->pose.pose.position.y; gps[i][j] = temps_.gps_data->latitude;}
                if (j ==2){wow(i,j)=1.0; this->temp.pop();}
            }
                                }
        // wow(0,0) = 0.;
        // wow(0,1) = 0.;
        // wow(0,2) = 1.;
        // wow(1,0) = 0.;
        // wow(1,1) = 1000.;
        // wow(1,2) = 1.;
        // wow(2,0) = 1000.;
        // wow(2,1) = 1000.;
        // wow(2,2) = 1.;
        // for(int i=0; i<3; i++){for (int j=0 ;j<3; j++){RCLCPP_INFO(get_logger(), "%f", gps2local_trans[i][j]);}}






        // for(int i=0; i<3; i++){for (int j=0 ;j<3; j++){gps2local_trans[i][j] = wow(i,j);}}
        for(int i=0; i<3; i++){for (int j=0 ;j<2; j++){RCLCPP_INFO(get_logger(), "gps is %.15lf", gps[i][j]);}}

        wow = wow.inverse();

        for(int i=0; i<3; i++){for (int j=0 ;j<3; j++){gps2local_trans[i][j] = wow(i,j);}}
        for(int i=0; i<3; i++){for (int j=0 ;j<3; j++){RCLCPP_INFO(get_logger(), "inverse is %f", gps2local_trans[i][j]);}}

        this->make_transform_ok =true;

            if(this->make_transform_ok)
            {

            Eigen::MatrixXd gps_eigen(3,2);
            Eigen::MatrixXd output(3,2);
            Eigen::MatrixXd B_dot(2,2);

            gps_eigen.setZero();
            output.setZero();
            B_dot.setZero();

            for(int i=0; i<3; i++){for (int j=0; j<2; j++){gps_eigen(i,j)=gps[i][j];}}
            // gps_eigen(0,0) = 1.0;
            // gps_eigen(0,1) = 1.0;
            // gps_eigen(1,0) = 1.0;
            // gps_eigen(1,1) = 1.0001;
            // gps_eigen(2,0) = 1.0001;
            // gps_eigen(2,1) = 1.0001;

            // for(int i=0; i<3; i++){for (int j=0 ;j<2; j++){RCLCPP_INFO(get_logger(), "gps is %lf", gps[i][j]);}}
            output = wow * gps_eigen;
            gps_params[0] = output(0,0);
            gps_params[3] = output(0,1);
            gps_params[1] = output(1,0);
            gps_params[4] = output(1,1);
            gps_params[2] = output(2,0);
            gps_params[5] = output(2,1);

            // B_dot(0,0) = gps_params[0];
            // B_dot(0,1) = gps_params[1];
            // B_dot(1,0) = gps_params[3];
            // B_dot(1,1) = gps_params[4];
            // B_dot = B_dot.inverse();
            // gps_params[0] = B_dot(0,0);
            // gps_params[1] = B_dot(0,1);
            // gps_params[3] = B_dot(1,0);
            // gps_params[4] = B_dot(1,1);
             for(int i=0; i<6 ;i++){RCLCPP_INFO(get_logger(), "gps_params %d is %.15lf", i,gps_params[i]);}
            }
}

}//namespace
