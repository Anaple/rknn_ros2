// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/

// ROS2 Node

#include <stdio.h>
#include <sys/time.h>
#include <thread>
#include <queue>
#include <vector>
#include <iostream>
#include <filesystem>
#include <algorithm>
#define _BASETSD_H
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "rknnPool.hpp"
#include "ThreadPool.hpp"
#include "postprocess.h"
#include <msg_interfaces/msg/rgb_camera_obstacle.hpp>

using std::queue;
using std::time;
using std::time_t;
using std::vector;

class ImageSubscriberNode : public rclcpp::Node
{
public:
  ImageSubscriberNode() : Node("image_subscriber"), tdpool(6)
  {
    // 订阅 /color/raw 话题，回调函数为 processImage
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10, std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1));
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
    rgb_obstacle_publisher_ = create_publisher<msg_interfaces::msg::RGBCameraObstacle>("rgb_camera/obstacle", 10);

    // 订阅深度图像和相机信息
    depth_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", 10,
        std::bind(&ImageSubscriberNode::depthImageCallback, this, std::placeholders::_1));

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", 10,
        std::bind(&ImageSubscriberNode::cameraInfoCallback, this, std::placeholders::_1));

    model_name = "install/rknn_2/share/rknn_2/model/RK3588/yolov5s-640-640.rknn"; // 参数二，模型所在路径
    // Create a MultiThreadedExecutor with the specified number of threads
    printf("模型名称:\t%s\n", model_name);
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
    catch (const cv::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void processImage(const cv::Mat &image)
  {
    // Perform your image processing here using RKNN or other methods
    // ...
    initRkPool(image);
    if (count > n)
    {
      // 发布帧
      futs.front().get();
      futs.pop();
      cv::Mat cvImage = rkpool[frames % n]->ori_img;
      sensor_msgs::msg::Image::SharedPtr rosImage = std::make_shared<sensor_msgs::msg::Image>();
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cvImage).toImageMsg(*rosImage);
      rgb_obstacle_publisher_->publish(rkpool[frames % n]->rgb_frames_data);
      image_publisher_->publish(*rosImage);
      // 更新帧
      rkpool[frames % n]->ori_img = cv::Mat(image);
      rkpool[frames % n]->depth_image_msg_ptr = depth_image_msg_gb;
      msg_interfaces::msg::RGBCameraObstacle::SharedPtr new_rgb = std::make_shared<msg_interfaces::msg::RGBCameraObstacle>();
      rkpool[frames % n]->rgb_frames_data = new_rgb;
      futs.push(tdpool.submit(&rknn_lite::interf, &(*rkpool[frames++ % n])));
    }
  }

  void initRkPool(const cv::Mat &image)
  {
    if (count < n)
    {
      auto rknn_model = std::make_shared<rknn_lite>(model_name, count % 3);
      rkpool.push_back(rknn_model);
      rknn_model->ori_img = cv::Mat(image); // Assuming mat_ptr is available
      rkpool[frames % n]->depth_image_msg_ptr = depth_image_msg_gb;
      msg_interfaces::msg::RGBCameraObstacle new_rgb;
      rkpool[frames % n]->rgb_frames_data = new_rgb;
      futs.push(tdpool.submit(&rknn_lite::interf, rknn_model.get()));
      count++;
    }
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

  float getDepthAtPixel(const sensor_msgs::msg::Image::SharedPtr depth_image_msg, int x, int y)
  {
    // 获取深度图像的宽度和高度
    int width = depth_image_msg->width;
    int height = depth_image_msg->height;

    // 假设深度图像数据类型是单通道16位无符号整数，单位是毫米
    uint16_t *depth_data = reinterpret_cast<uint16_t *>(&depth_image_msg->data[0]);

    // 计算深度图像中的索引
    int index = y * width + x;

    // 获取深度值
    uint16_t depth_value = depth_data[index];

    return static_cast<float>(depth_value);
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  sensor_msgs::msg::Image::SharedPtr depth_image_msg_gb;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<msg_interfaces::msg::RGBCameraObstacle>::SharedPtr rgb_obstacle_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;

  char *model_name = NULL;
  int n = 6, frames = 0; // 设置线程数
  int count = 0;

  std::vector<std::shared_ptr<rknn_lite>> rkpool;
  std::queue<std::future<int>> futs;
  dpool::ThreadPool tdpool;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
