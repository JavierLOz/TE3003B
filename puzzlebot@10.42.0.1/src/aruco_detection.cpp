#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include<opencv2/imgproc/imgproc.hpp> 
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Aruco_Detection : public rclcpp::Node
{
  
  private:
    cv::Mat markerImage;
     // List of each detected marker  
    std::vector<int> markerIds;
    // list of corners of the markers detected (clockwise startig top left)
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    
    // Initiate the detector parameters
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    // Initiate predefined aruco dictionary
    // cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    cv::aruco::ArucoDetector detector = cv::aruco::ArucoDetector(dictionary, detectorParams);

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    
    // Initiate image 
   // why are callbacks privates
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
       
        cv_bridge::CvImagePtr cv_ptr;
        //cv_ptr->image
        try
        {
            // TODO: set?
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

    detector.detectMarkers(cv_ptr->image, markerCorners, markerIds, rejectedCandidates);
    // Draw detections  
    cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);

    // Define interest id 
    int search_id = 124;

    // Define detected state
    bool detected = 0;

    // Store detected corners
    std::vector<cv::Point2f> detected_aruco;

    // Get the corners of the marker with a specific id 
    for (int i=0; i < markerIds.size();i++){
        if (markerIds[i] == search_id ){
            detected_aruco = markerCorners[i];
            detected = 1;
        }
    }

    if (detected){
        // compute center
        cv::Point2f center;
        center = middle_point(detected_aruco[0],detected_aruco[2]);
        cv::circle(cv_ptr->image, center,7, cv::Scalar(255, 0, 0), 2);

        std::cout << "aruco with id: " << search_id << " was detected on: (" << center.x << "," << center.y << ")" << std::endl;
    }

    // publish edited image
    pub_.publish(cv_ptr->toImageMsg());
    // cv::imwrite("../out.jpg", cv_ptr->image);


    std::cout << "id: " << markerIds[0] << std::endl;
        }

  public:
    Aruco_Detection(): Node("Aruco_Detection")
    {
      rmw_qos_profile_t custom_qos = rmw_qos_profile_default;

      sub_ = image_transport::create_subscription(this, "/video_source",
      std::bind(&Aruco_Detection::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
      
      pub_ = image_transport::create_publisher(this, "out_image", custom_qos);

    }

    cv::Point2f middle_point(cv::Point2f point1, cv::Point2f point2){
      // Initialize return vector 
      cv::Point2f mdl_point;
      // x coord
      mdl_point.x = ((point2.x+point1.x) / 2); 
      mdl_point.y = ((point2.y+point1.y) / 2); 

      return mdl_point;

    
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Aruco_Detection>());
  rclcpp::shutdown();
  return 0;
}