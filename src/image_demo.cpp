
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


using namespace std::chrono_literals;

class Image : public rclcpp::Node {
public:
    Image(std::string path) : 
        Node("axes_node") {
            
            // Load the image file from disk.
            std::string image_path = cv::samples::findFile(path);
            cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR); 
            
            // Show it and wait for button press.
            cv::imshow("Window", img);
            cv::waitKey(0);

            // Instead of manually creating the sensor_msgs::msgs::Image, 
            // we utilize the cv_bridge library.
            cv_bridge::CvImage img_bridge;
            sensor_msgs::msg::Image img_msg; 
            img_msg.header.stamp = this->now();
            img_msg.header.frame_id = "map"; 
            // This adds the image with the specified encoding the the message 
            // with the specified header.
            img_bridge = cv_bridge::CvImage(img_msg.header,  
                sensor_msgs::image_encodings::BGR8, img);
            img_bridge.toImageMsg(img_msg);

            // And we finally publish it.
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("lena", 10);
                   
            publisher_->publish(img_msg);
        }
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto axes_node = std::make_shared<Image>("/home/grans/dev_ws/src/rviz_demo/resources/lena.png");
    rclcpp::spin(axes_node);
    rclcpp::shutdown();
    return 0;
}

