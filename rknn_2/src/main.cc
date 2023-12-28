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
    mat_ptr = image;
    initRkPool();
    if (count > n)
    {
      if (futs.front().get() != 0)
        printf("0");
      futs.pop();
      cv::Mat cvImage = rkpool[frames % n]->ori_img;
      rkpool[frames % n]->ori_img = cv::Mat(mat_ptr);
      detect_result_group_t detect_result_group = rkpool[frames % n]->result;
      msg_interfaces::msg::RGBCameraObstacle rgb_frames_data;
      char text[256];
      for (int i = 0; i < detect_result_group.count; i++)
      {
        detect_result_t *det_result = &(detect_result_group.results[i]);

        int x1 = det_result->box.left;
        int y1 = det_result->box.top;
        int x2 = det_result->box.right;
        int y2 = det_result->box.bottom;
        int w = x2 - x1;
        int h = y2 - y1;
        double deep = depthImageRecv(depth_image_msg_gb, y1, y2, x1, x2);
        // printf("%s %.1f%% w: %d h: %d\n", det_result->name, det_result->prop * 100, w, h);
        rgb_frames_data.obstacle_type.push_back(det_result->name);
        rgb_frames_data.obstacle_position.push_back(x1);
        rgb_frames_data.obstacle_position.push_back(y1);
        rgb_frames_data.obstacle_position.push_back(x2);
        rgb_frames_data.obstacle_position.push_back(y2);

        sprintf(text, "%s %.1f%%  deepth: %.1f m", det_result->name, det_result->prop * 100, deep / 1000);
        putText(cvImage, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

        // 640 to 1280
        rgb_frames_data.obstacle_size.push_back(deep);
      }

      rgb_obstacle_publisher_->publish(rgb_frames_data);
      sensor_msgs::msg::Image::SharedPtr rosImage = std::make_shared<sensor_msgs::msg::Image>();
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cvImage).toImageMsg(*rosImage);
      image_publisher_->publish(*rosImage);
      futs.push(tdpool.submit(&rknn_lite::interf, &(*rkpool[frames++ % n])));
    }
    count++;
    // For demonstration, display the processed image
  }

  void initRkPool()
  {
    if (count < n)
    {
      auto rknn_model = std::make_shared<rknn_lite>(model_name, count % 3);
      rkpool.push_back(rknn_model);
      rknn_model->ori_img = cv::Mat(mat_ptr); // Assuming mat_ptr is available
      futs.push(tdpool.submit(&rknn_lite::interf, rknn_model.get()));
    }
  }
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr depth_image_msg)
  {
    depth_image_msg_gb = depth_image_msg;
  }

  float depthImageRecv(const sensor_msgs::msg::Image::SharedPtr depth_image_msg, int top, int bottom, int left, int right)
  {
    std::map<float, int> depth_counts;

    for (int y = top; y <= bottom; ++y)
    {
      for (int x = left; x <= right; ++x)
      {
        // 获取深度值
        float depth = getDepthAtPixel(depth_image_msg, x, y);

        if (depth > 0)
        {
          // 将深度值添加到映射中
          depth_counts[depth]++;
        }
      }
    }

    float mode_depth = findMode(depth_counts);

    // 在这里使用众数的深度值...

    RCLCPP_INFO(this->get_logger(), "Mode depth: %f", mode_depth);
    return mode_depth;
  }

  float findMode(const std::map<float, int> &depth_counts)
  {
    // 找到映射中出现次数最多的深度值
    auto mode_iterator = std::max_element(
        depth_counts.begin(), depth_counts.end(),
        [](const auto &a, const auto &b)
        { return a.second < b.second; });

    // 返回众数深度值
    return mode_iterator->first;
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

  cv::Mat mat_ptr;
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
