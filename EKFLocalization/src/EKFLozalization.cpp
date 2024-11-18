// TODO include packages in the cmake and package
// TODO declare object variables as paramters

// https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom 
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

constexpr double pi = 3.14159265358979323846;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EKFLocalization : public rclcpp::Node
{
  
  private:
    // Initialize object values 
    float dt;
    float sample_time = 0.2;    
    rclcpp::Time start_time;
    rclcpp::Time last_time;
    rclcpp::Time current_time;

    // initialize robot properties 
    float wheel_rad  = 0.05;
    float wheel_dist = 0.19;

    bool first = 1;

    // Initialize robot measurements
    float wR  = 0.0;
    float wL  = 0.0;
    float v_k = 0.0;
    float w_k = 0.0;

    float delta_zx = 0.0;
    float delta_zy = 0.0;
    float p        = 0.0;
    // Noise parameters
    float kR = 0.0;
    float kL = 0.0;

    // Sensor state
    float r_rho   = 0.0;
    float z_alpha = 0.0;
    float z_rho   = 0.0;    

    // Set initial sensor readings
    Eigen::Vector2f z_measured = Eigen::Vector2f::Zero(); 
    Eigen::Vector2f z_static = Eigen::Vector2f::Zero(); 

    // Set robot initial state
    Eigen::Vector3f s      = Eigen::Vector3f::Zero();
    Eigen::Vector3f prev_s = Eigen::Vector3f::Zero();

    // Initializa filter parameters
    Eigen::Matrix3f H          = Eigen::Matrix3f::Identity(); //Robot Pose Model Jacobian Matrix
    Eigen::Matrix3f Sigma      = Eigen::Matrix3f::Zero(); //Current Robot Pose Model Covariance Matrix
    Eigen::Matrix3f prev_Sigma = Eigen::Matrix3f::Zero(); //Previous Robot Pose Model Covariance Matrix
    Eigen::Matrix3f Q          = Eigen::Matrix3f::Zero(); //Current Robot Pose Noise Covariance Matrix

    Eigen::Matrix2f SigmaD = Eigen::Matrix2f::Zero(); //Wheel Noise Covariance Matrix
    Eigen::MatrixXf Nabla  = Eigen::MatrixXf::Zero(3, 2); //Wheel Model Noise Jacobian Matrix

    // Initialize robot states
    Eigen::Vector2f z = Eigen::Vector2f::Zero();    // Corrected Robot Pose Observation
    Eigen::MatrixXf G = Eigen::MatrixXf::Zero(2, 3);// Current Robot Pose Observation Jacobian Matrix
    Eigen::Matrix2f Z = Eigen::Matrix2f::Zero();    // Robot Pose Observation Covariance Matrix
    Eigen::Matrix2f R = Eigen::Matrix2f::Zero();    // Current Robot Observation Noise Covariance Matrix
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(3,2); // Kalman Gain

    // Initialize odom message
    nav_msgs::msg::Odometry odometry_msg;

    // Initialize transformed message 
    geometry_msgs::msg::TransformStamped odom_trans;

    // Initialize subscribers and publishers 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subsVelR_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subsVelL_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subsCont_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pubOdom_;

    // Initialize timer 
    rclcpp::TimerBase::SharedPtr timer_;

    // Initialize broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

    //**-----------------------Subscribers------------------------- */
    // Store encoder vel (R)
    void encR_callback(const std_msgs::msg::Float32::SharedPtr msg) {  
      
      this->wR = msg->data;
      // RCLCPP_INFO(this->get_logger(), "wR: '%f'", this->wR);  

    }
    
    // Store encoder vel (L)
    void encL_callback(const std_msgs::msg::Float32::SharedPtr msg) {  
      
      this->wL = msg->data;

    }

    // Store last controll signal 
    void control_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {  

      v_k = msg->linear.x;
      w_k = msg->angular.z;

    }
    //**---------------------------------------------------------- */


    //**----------------------math functions---------------------- */
    float wrap_to_pi(float& theta){
      if (theta > pi){
        theta = theta - 2 * pi;
      }
      else if (theta < -pi){
        theta = theta + 2 * pi;
      }
      return theta;
    }
    
    float compute_linear_vel(){
      float v = wheel_rad * (this->wR + this->wL) / 2;
      return v;
    }

    float compute_angular_vel(){
      float w = wheel_rad * (this->wR - this->wL) / this->wheel_dist;
      return w;
    }
    //**------------------------------------------------------------ */

    //**---------------------Kalman functions ---------------------- */
    void calc_gradient_w(){
        Nabla(0,0) =  cos(prev_s(2));
        Nabla(0,1) =  cos(prev_s(2));
        Nabla(1,0) =  sin(prev_s(2));
        Nabla(1,1) =  sin(prev_s(2));
        Nabla = 0.5 * wheel_rad * dt * Nabla;
    }

    void calc_SigmaD(){
      SigmaD(0,0) = kR * std::abs(wR);
      SigmaD(1,1) = kL * std::abs(wL);
    }

    void estimate_Q(){
      calc_gradient_w();
      calc_SigmaD();
      Q = Nabla * SigmaD * Nabla.transpose(); 
    }

    void calc_miuHat(){
      s(0) = prev_s(0) + dt * v_k * cos(prev_s(2));
      s(1) = prev_s(1) + dt * v_k * sin(prev_s(2));
      s(2) = prev_s(2) + dt * w_k;
      std::cout << "v_k: " << v_k << std::endl;
      std::cout << "w_k: " << w_k << std::endl;
      std::cout << "s(0): " << s(0) << std::endl;
      std::cout << "s(1): " << s(1) << std::endl;
      std::cout << "s(2): " << s(2) << std::endl;

    }

    void calc_gradient_h(){
      H(0,2) = -dt * v_k * sin(prev_s(2));
      H(1,2) = dt * v_k * cos(prev_s(2));
    }

    void calc_SigmaHat(){
      Sigma = H * prev_Sigma * H.transpose() + Q;
      std::cout << "sigma_hat: " << Sigma << std::endl;

    }

    void calc_zHat(){
      delta_zx = z_static(0) - s(0);
      delta_zy = z_static(1) - s(1);
      // std::cout<< "deltazx: " << delta_zx << std::endl;
      // std::cout<< "deltazy: " << delta_zy << std::endl;
      // std::cout<< "Powdzy: " << pow(delta_zy,2) << std::endl;
      p = pow(delta_zx,2) + pow(delta_zy,2); 
      // std::cout<< "p: " << p << std::endl;

      float theta_ = atan2(delta_zy,delta_zx) - s(2);
      std::cout<< "theta_: " << theta_ << std::endl;
      z(0) = sqrt(p);
      // std::cout<< "sqrt(p: " << z(0) << std::endl;

      z(1) = wrap_to_pi(theta_);
    }

    void calc_gradient_g(){
        float sqrt_p = sqrt(p);
        if (sqrt_p == 0){
          G(0, 0) = 0;
          G(0, 1) = 0;
          G(0, 2) = 0.0;
          G(1, 0) = 0;
          G(1, 1) = 0;
          G(1, 2) = -1.0;
        }
        else{
          G(0, 0) = -delta_zx / sqrt(p);
          G(0, 1) = -delta_zy / sqrt(p);
          G(0, 2) = 0.0;
          G(1, 0) = delta_zy / p;
          G(1, 1) = -delta_zx / p;
          G(1, 2) = -1.0;
        }
    }

    void calc_Z(){
      Z = G * Sigma * G.transpose() + R;
    }

    void calc_kalman_gain(){
       std::cout << "k antes: " << K << std::endl;
       std::cout << "G antes: " << G << std::endl;
       std::cout << "sigam antes: " << Sigma << std::endl;
      K = Sigma * G.transpose() * Z.inverse();
    }

    void calc_miu(){
      // std::cout << "s antes: " << s << std::endl;
      // std::cout << "k antes: " << K << std::endl;
      // std::cout << "Zmes antes: " << z_measured << std::endl;
      // std::cout << "Z antes: " << z << std::endl;
      s = s + K * (z_measured - z);

    }

    void calc_Sigma(){
      Eigen::Matrix3f I_3x3 = Eigen::Matrix3f::Identity();
      Sigma = (I_3x3 - K * G) * Sigma;
    }
    //**------------------------------------------------------------- */

    //**----------------------runnig functions-------------------- */
    void publish_result(){
      // define header for TRANSFORMED 
      odom_trans.header.stamp = this->get_clock()->now();
      odom_trans.header.frame_id = "world";
      odom_trans.child_frame_id = "odom";
      
      // define header for ODOOM
      odometry_msg.header.stamp = this->get_clock()->now();
      odometry_msg.header.frame_id = "world";
      odometry_msg.child_frame_id = "odom";

      // define content for ODOM 
      odometry_msg.pose.pose.position.x = s(0); 
      odometry_msg.pose.pose.position.y = s(1);
      odometry_msg.pose.pose.position.z = wheel_rad;

      // define content for TRANSFORMED
      odom_trans.transform.translation.x = s(0); 
      odom_trans.transform.translation.y = s(1);
      odom_trans.transform.translation.z = wheel_rad;

      tf2::Quaternion q;
      q.setRPY(0, 0, s(2));
      // std::cout << "angle: " << s(2) << std::endl;
      odom_trans.transform.rotation.x = q.x();
      odom_trans.transform.rotation.y = q.y();
      odom_trans.transform.rotation.z = q.z();
      odom_trans.transform.rotation.w = q.w();

      odometry_msg.pose.pose.orientation.x = q.x();
      odometry_msg.pose.pose.orientation.y = q.y();
      odometry_msg.pose.pose.orientation.z = q.z();
      odometry_msg.pose.pose.orientation.w = q.w();

      odometry_msg.twist.twist.linear.x  = v_k;
      odometry_msg.twist.twist.angular.z = w_k;

      odometry_msg.pose.covariance[0]  = Sigma(0, 0);
      odometry_msg.pose.covariance[1]  = Sigma(0, 1);
      odometry_msg.pose.covariance[5]  = Sigma(0, 2);
      odometry_msg.pose.covariance[6]  = Sigma(1, 0);
      odometry_msg.pose.covariance[7]  = Sigma(1, 1);
      odometry_msg.pose.covariance[11] = Sigma(1, 2);
      odometry_msg.pose.covariance[30] = Sigma(2, 0);
      odometry_msg.pose.covariance[31] = Sigma(2, 1);
      odometry_msg.pose.covariance[35] = Sigma(2, 2);
      
      // Send the transformation
      tf_broadcaster_->sendTransform(odom_trans);
      // Send Odom
      pubOdom_->publish(odometry_msg);    
    }

    void set_Previous(){
      prev_s(0) = s(0);
      prev_s(1) = s(1);
      prev_s(2) = s(2);
      prev_Sigma = Sigma;
      last_time = current_time;
    }

    void PoseEstimation(float dt_, bool landmark_status){
        
      // TODO  -> Obtener del subscriptor del pose. 
      z_measured(0) = 0.0; //landmark_measured(0);
      z_measured(1) = 0.0; //landmark_measured(1);

      // TODO  Definir al iniciar 
      z_static(0) = 0.0;// landmark_pos(0);
      z_static(1) = 0.0; //landmark_pos(1);

      // TODO -> revisar estas de dnde vienne -> del subscriptor??
      // v_k = control_u[0];
      // w_k = control_u[1];

      // TODO > obtener del subscriptor
      // wl = measured_wl;
      // wr = measured_wr;

      kL = 0.0; //calibrated_klr(0);
      kR = 0.0; //calibrated_klr(1);

      R(0,0) = 0.1;  
      R(1,1) = 0.2;  

      //calibrated_R;
      dt = dt_;

      estimate_Q();
      Q(0, 0) = 0.001; Q(0, 1) = 0.001; Q(0, 2) = 0.001;
      Q(1, 0) = 0.001; Q(1, 1) = 0.001;  Q(1, 2) = 0.001;
      Q(2, 0) = 0.001; Q(2, 1) = 0.001; Q(2, 2) = 0.001;

      calc_miuHat(); // Ideal calculated pose
      calc_gradient_h(); // Pose Model linerization
      calc_SigmaHat(); // Uncertainty of Ideal calculated pose
      if (landmark_status){
        calc_zHat(); // Ideal observed pose
        calc_gradient_g() ;// Observation Model linerization
        calc_Z() ;// Uncertainty of Ideal observed pose
        calc_kalman_gain(); // Kalman Filter Gain!
        calc_miu(); // Estimated pose
        calc_Sigma() ;// Estimated uncertainty of pose
      }
      set_Previous();
    }

    void timer_callback(){
      if (first){
        first = 0;
        start_time = this->get_clock()->now();
        current_time = this->get_clock()->now();
        last_time = this->get_clock()->now();
        // TODO : Fix Nabla initialization. Error when assigning manually as param
        Nabla(2,0) =  2.0/0.19;
        Nabla(2,1) =  2.0/0.19;
        std::cout << "Nabla: " << Nabla <<std::endl;
      }
      else {
        current_time = this->get_clock()->now();
        float dt_ = current_time.seconds() - last_time.seconds();
        if (dt_ >= sample_time){
          std::cout << "dt: " << dt_ << std::endl;
          PoseEstimation(dt_,false);
          publish_result();
          // last_time = this->get_clock()->now();
        }
      }
    }
  //**------------------------------------------------------------------- */
  public:
    EKFLocalization()
    : Node("EKFLozalization")
    {
      
      rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
      qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

      subsVelR_ = this->create_subscription<std_msgs::msg::Float32>(
      "/VelocityEncR", qos_settings, std::bind(&EKFLocalization::encR_callback, this, std::placeholders::_1));

      subsVelL_ = this->create_subscription<std_msgs::msg::Float32>(
      "/VelocityEncL", qos_settings, std::bind(&EKFLocalization::encL_callback, this, std::placeholders::_1));

      subsCont_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos_settings, std::bind(&EKFLocalization::control_callback, this, std::placeholders::_1));

      pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/pzbt_odom",10);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      timer_ = this->create_wall_timer(
      100ms, std::bind(&EKFLocalization::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFLocalization>());
  rclcpp::shutdown();
  return 0;
}
