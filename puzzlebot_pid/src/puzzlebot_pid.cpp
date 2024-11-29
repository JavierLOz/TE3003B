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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Puzzlebot_PID : public rclcpp::Node
{
  
  private:
    
    // Initialize object values 
    bool first = 1;
    float sample_time = 0.2;

    // Set controller parameters for linear speed
    float kp_r = 0.02;
    float ki_r = 0.02;
    float kd_r = 0.02;

    // Set controller parameters for "angular_distance"
    float kp_t = 0.1;
    float ki_t = 0.1;
    float kd_t = 0.1;

    // Set variables for control procedure 
		float derivative_signal_r   = 0.0;
		float integral_signal_r     = 0.0;
		float proportional_signal_r = 0.0;

		float derivative_signal_t   = 0.0;
		float integral_signal_t     = 0.0;
		float proportional_signal_t = 0.0;

    float past_deriv_r = 0.0;
		float past_deriv_t = 0.0;

    // Initialize values for sensor reading 
    float sensed_distance = 0;
    float sensed_angle    = 0;

    // Initialize time variables
    rclcpp::Time start_time;
    rclcpp::Time last_time;
    rclcpp::Time current_time;

    // Assign frames for tf listener 
    std::string fromFrameRel = "odom";
    std::string toFrameRel = "world";

    // Initialize setpoint
    geometry_msgs::msg::Pose set_point;
    float set_point_angle = 0;

    // Initialize controller output
    geometry_msgs::msg::Twist cmd_msg;

    // Initialize transformed message 
    geometry_msgs::msg::TransformStamped trans_msg;
    float trans_angle = 0;

    // Initialize broadcaster
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    // Initialize buffer to store data from transformed listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // Initialize subscribers and publishers 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_msg;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subs_set_point;

    // Initialize timer 
    rclcpp::TimerBase::SharedPtr timer_;

    //**------------------------------------ callback functions----------------------------------------------- */
    bool get_tf(){
      try {
        
        trans_msg = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
        
        trans_angle = wrap_to_pi(arctan2((trans_msg.transform.translation.y - 0), (trans_msg.transform.translation.x - 0)));

        return true;

      } catch (const tf2::TransformException & ex) {
          //RCLCPP_INFO(
          //this->get_logger(), "Could not transform %s to %s: %s",
          //toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return false;
      }
    
    }

    void set_point_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {  
      
      set_point.position = msg->position;
      set_point.orientation = msg->orientation;

      set_point_angle = wrap_to_pi( arctan2( (set_point.position.y - 0), (set_point.position.x - 0) ) );

    }

  //**---------------------------------------------------------------------------------------------------- */

  //**--------------------------------------Math Functions------------------------------------------------- */
  float arctan2(float y, float x){
      float angle = atan2(y,x);
      if (x < 0){
        angle += (y >= 0) ? M_PI : M_PI;
      }
      return angle;
    }
    
    float wrap_to_pi(float theta){
      if (theta > M_PI){
        theta = theta - 2 * M_PI;
      }
      else if (theta < -M_PI){
        theta = theta + 2 * M_PI;
      }
      return theta;
    }
  //**---------------------------------------------------------------------------------------------------- */

  //**------------------------------------ running functons------------------------------------------------ */

    void PD_control(float dt){
    
	    get_tf();
		
      float r_delta = sqrt( pow((set_point.position.x - trans_msg.transform.translation.x),2) + 
                            pow((set_point.position.y - trans_msg.transform.translation.y),2) );
      
      float t_delta = wrap_to_pi(set_point_angle - trans_angle);

      std::cout << "r_delta: " << r_delta << std::endl;
      std::cout << "t_delta: " << t_delta << std::endl;

      // double roll, pitch, yaw_delta;
      

      // tf2::Quaternion q_trans(trans_msg.transform.rotation.x, 
      //                         trans_msg.transform.rotation.y, 
      //                         trans_msg.transform.rotation.z, 
      //                         trans_msg.transform.rotation.w);

      // tf2::Quaternion q_set(set_point.orientation.x, 
      //                       set_point.orientation.y, 
      //                       set_point.orientation.z, 
      //                       set_point.orientation.w);

      // tf2::Quaternion q_delta;

      // q_delta = q_set * q_trans.inverse();

      // tf2::Matrix3x3 m(q_delta);
      // m.getRPY(roll, pitch, yaw_delta);
      
      proportional_signal_r = (r_delta) * kp_r;
      derivative_signal_r   = ((r_delta - past_deriv_r) / dt) * kd_r;

      proportional_signal_t = (t_delta) * kp_t;
      derivative_signal_t   = ((t_delta - past_deriv_t) / dt) * kd_t;
      
    }

    void run_node(float dt){
      PD_control(dt);      

      float controlled_t = proportional_signal_t + derivative_signal_t;  
      float controlled_r = proportional_signal_r + derivative_signal_r ; 

      cmd_msg.linear.x = controlled_r;
      cmd_msg.linear.y = 0;
      cmd_msg.linear.z = 0;

      cmd_msg.angular.x = 0;
      cmd_msg.angular.y = 0;
      cmd_msg.angular.z = controlled_t;

      std::cout << "controlled r: " << controlled_r << std::endl;
      std::cout << "controlled t: " << controlled_t << std::endl;


      pub_cmd_msg->publish(cmd_msg);

      past_deriv_r = derivative_signal_r;
      past_deriv_t = derivative_signal_t;

    }

    void timer_callback(){
      if (this->first){
        first = 0;
        trans_msg.transform.translation.x = 0;
        trans_msg.transform.translation.y = 0;
        trans_msg.transform.translation.z = 0;

        trans_msg.transform.rotation.x = 0; 
        trans_msg.transform.rotation.y = 0;
        trans_msg.transform.rotation.z = 0; 
        trans_msg.transform.rotation.w = 1;
        start_time = this->get_clock()->now();
        current_time = this->get_clock()->now();
        last_time = this->get_clock()->now();
      }
      else {
        current_time = this->get_clock()->now();
        float dt = current_time.seconds() - last_time.seconds();
        if (dt >= this->sample_time){
          std::cout << "dt: " << dt << std::endl;
          run_node(dt);
          last_time = this->get_clock()->now();
        }
      }
    }

  public:
    Puzzlebot_PID()
    : Node("puzzlebot_pid")
    {
      
      rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
      qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

	    subs_set_point = this->create_subscription<geometry_msgs::msg::Pose>(
      "/set_point", qos_settings, std::bind(&Puzzlebot_PID::set_point_callback, this, std::placeholders::_1));

      pub_cmd_msg = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

	      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 

      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  

      timer_ = this->create_wall_timer(
      10ms, std::bind(&Puzzlebot_PID::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Puzzlebot_PID>());
  rclcpp::shutdown();
  return 0;
}
