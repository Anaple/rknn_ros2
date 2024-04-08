#include <stdio.h>
#include <memory>
#include <sys/time.h>

#include "rknnPool.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber")
    {
        int init = poolPtr.init();
        if (init != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "rknn Pool init fail");
        }
        // 订阅 /color/raw 话题，回调函数为 processImage
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", rclcpp::QoS(5).best_effort(), std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1));
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image_topic", rclcpp::QoS(1).best_effort());
        // 订阅深度图像和相机信息
        depth_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", rclcpp::QoS(5).best_effort(),
            std::bind(&ImageSubscriberNode::depthImageCallback, this, std::placeholders::_1));

        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/depth/camera_info", 10,
            std::bind(&ImageSubscriberNode::cameraInfoCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "rknn node start");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS Image message to OpenCV image
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            // Process the image (e.g., perform inference)
            processImage(cv_ptr->image);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
        }
    }
    void processImage(const cv::Mat &image)
    {

        poolPtr.put(image);
        cv::Mat modifiableImage = image; // Create a modifiable copy

        if (count > 3)
        {
            poolPtr.get(modifiableImage);
        }
        else
        {
            count++;
        }

        sensor_msgs::msg::Image::SharedPtr rosImage = std::make_shared<sensor_msgs::msg::Image>();
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", modifiableImage).toImageMsg(*rosImage);
        image_publisher_->publish(*rosImage);
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr depth_image_msg)
    {
        depth_image_msg_gb = depth_image_msg;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
    {
        // 在这里处理相机信息，可以使用camera_info_msg等
        // 获取相机的内参矩阵等信息，用于坐标变换
        // 这里假设你已经实现了获取相机内参矩阵的函数 getCameraMatrix(camera_info_msg)，
        // 具体实现方式可以参考 sensor_msgs::msg::CameraInfo 文档。
    }
    int count = 0;
    int frames = 0;
    cv_bridge::CvImageConstPtr cv_ptr;
    sensor_msgs::msg::Image::SharedPtr depth_image_msg_gb;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rknnPool poolPtr = rknnPool("/home/umsrock5a/model/yolov5.rknn", 3);
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());
    rclcpp::shutdown();
    return 0;
};