// TODO include packages in the cmake and package
// TODO declare object variables as paramters

// https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom 
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Puzzlebot_Odometry : public rclcpp::Node
{
  
  private:
    
    // Velocity Float64
    // /VelocityEncL
    // /VelocityEncR

    // Initialize object values 

    bool first = 1;

    float wR = 0;
    float wL = 0;

    float wheel_rad = 0.05;
    float wheel_dist = 0.19;

    float robot_x = 0;
    float robot_y = 0;
    float robot_z = 0;
    float robot_theta = 0;

    rclcpp::Time start_time;
    rclcpp::Time last_time;
    rclcpp::Time current_time;
    float sample_time = 0.2;

    // Initialize odom message
    nav_msgs::msg::Odometry odometry_msg;

    // Initialize transformed message 
    geometry_msgs::msg::TransformStamped trans_msg;

    // Initialize subscribers and publishers 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subsVelR_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subsVelL_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pubOdom_;

    // Initialize timer 
    rclcpp::TimerBase::SharedPtr timer_;

    // Initialize broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Store encoder vel (R)
    void encR_callback(const std_msgs::msg::Float32::SharedPtr msg) {  
      
      this->wR = msg->data;
      // RCLCPP_INFO(this->get_logger(), "wR: '%f'", this->wR);  

    }
    
    // Store encoder vel (L)
    void encL_callback(const std_msgs::msg::Float32::SharedPtr msg) {  
      
      this->wL = msg->data;

    }

    float compute_linear_vel(){
      float v = this->wheel_rad * (this->wR + this->wL) / 2;
      return v;
    }

    float compute_angular_vel(){
      float w = this->wheel_rad * (this->wR - this->wL) / this->wheel_dist;
      return w;
    }

    void run_node(float dt){
      // compute angluar velocity 
      float w = compute_angular_vel();
      // compute linear velocity 
      float v = compute_linear_vel();

      float theta = w * dt;

      robot_theta += theta;

      float x_vel = v * cos(robot_theta);
      float y_vel = v * sin(robot_theta);

      robot_x += x_vel * dt;
      robot_y += y_vel * dt;
      robot_z = this->wheel_rad;
      // no hago modulo. 
     

      // define header for TRANSFORMED 
      this->trans_msg.header.stamp = this->get_clock()->now();
      this->trans_msg.header.frame_id = "world";
      this->trans_msg.child_frame_id = "odom";
      
      // define header for ODOOM
      this->odometry_msg.header.stamp = this->get_clock()->now();
      this->odometry_msg.header.frame_id = "world";
      this->odometry_msg.child_frame_id = "odom";

      // define content for ODOM 
      this->odometry_msg.pose.pose.position.x = robot_x; 
      this->odometry_msg.pose.pose.position.y = robot_y;
      this->odometry_msg.pose.pose.position.z = robot_z;

      // define content for TRANSFORMED
      this->trans_msg.transform.translation.x = robot_x;  
      this->trans_msg.transform.translation.y = robot_y;
      this->trans_msg.transform.translation.z = robot_z;

      tf2::Quaternion q;
      q.setRPY(0, 0, this->robot_theta);
      this->trans_msg.transform.rotation.x = q.x();
      this->trans_msg.transform.rotation.y = q.y();
      this->trans_msg.transform.rotation.z = q.z();
      this->trans_msg.transform.rotation.w = q.w();

      // std::cout<< "q.x: " << q.x() << std::endl; 
      // std::cout<< "q.y: " << q.y() << std::endl; 
      // std::cout<< "q.z: " << q.z() << std::endl; 
      // std::cout<< "q.w: " << q.w() << std::endl; 

      // RCLCPP_INFO(this->get_logger(), "q: '%f'", q);  
      // TODO: Aplicar modulo 
      // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_theta);
      this->odometry_msg.pose.pose.orientation.x = q.x();
      this->odometry_msg.pose.pose.orientation.y = q.y();
      this->odometry_msg.pose.pose.orientation.z = q.z();
      this->odometry_msg.pose.pose.orientation.w = q.w();

      // mal: this->odometry_msg.pose.pose.orientation.z = robot_theta;

      this->odometry_msg.twist.twist.linear.x  = x_vel;
      this->odometry_msg.twist.twist.linear.y  = y_vel;  

      this->odometry_msg.twist.twist.angular.z = w;
      
      // Send the transformation
      tf_broadcaster_->sendTransform(this->trans_msg);
      // Send Odom
      pubOdom_->publish(this->odometry_msg);    
    }

    void timer_callback(){
      if (this->first){
        first = 0;
        start_time = this->get_clock()->now();
        current_time = this->get_clock()->now();
        last_time = this->get_clock()->now();
      }
      else {
        current_time = this->get_clock()->now();
        float dt = current_time.seconds() - last_time.seconds();
        if (dt >= this->sample_time){
          this->run_node(dt);
          last_time = this->get_clock()->now();
        }
      }
    }

  public:
    Puzzlebot_Odometry()
    : Node("puzzlebot_odometry")
    {
      
      rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
      qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

      subsVelR_ = this->create_subscription<std_msgs::msg::Float32>(
      "/VelocityEncR", qos_settings, std::bind(&Puzzlebot_Odometry::encR_callback, this, std::placeholders::_1));

      subsVelL_ = this->create_subscription<std_msgs::msg::Float32>(
      "/VelocityEncL", qos_settings, std::bind(&Puzzlebot_Odometry::encL_callback, this, std::placeholders::_1));

      pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/pzbt_odom",10);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      timer_ = this->create_wall_timer(
      100ms, std::bind(&Puzzlebot_Odometry::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Puzzlebot_Odometry>());
  rclcpp::shutdown();
  return 0;
}