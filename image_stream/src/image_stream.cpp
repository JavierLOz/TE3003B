#include <stdlib.h>
#include <functional>
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
using namespace std::chrono_literals;

class Image_Stream : public rclcpp::Node
{
  
  private:
    // Initialize image 
    cv::Mat image;

    // Initialize message
    sensor_msgs::msg::Image img_msg;

    //Initialize cvbridge
    cv_bridge::CvImage img_bridge;
    
    // Initialize image publisher 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    // Initialize timer 
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback(){

      std_msgs::msg::Header header; // empty header
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, this->image);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      publisher_->publish(img_msg);
    }

  public:
    Image_Stream(): Node("Image_Stream")
    {

      this->image = cv::imread("/home/javier/Documents/cpp_files/aruco_detection/aruco_test.jpeg");
      
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

      timer_ = this->create_wall_timer(
      500ms, std::bind(&Image_Stream::timer_callback, this));

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Image_Stream>());
  rclcpp::shutdown();
  return 0;
}