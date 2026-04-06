#ifndef VISUAL_MODULE_H
#define VISUAL_MODULE_H

#include "RemoteAPIClient.h"
#include "opencv2/opencv.hpp"
#include "thread"
#include "iostream"

using namespace std;

// 全局变量，用于线程控制
static thread vision_thread_rgb;
static atomic<bool> vision_thread_rgb_running{false};
static atomic<bool> vision_thread_rgb_stop{false};

static thread vision_thread_depth;
static atomic<bool> vision_thread_depth_running{false};
static atomic<bool> vision_thread_depth_stop{false};

vector<float> get_sha();

cv::Mat get_rgb_picture(RemoteAPIObject::sim& sim, int visionSensorHandle);

cv::Mat get_share_rgb_picture();

cv::Mat get_depth_picture(RemoteAPIObject::sim& sim, int visionSensorHandle);

cv::Point3f detectObject3D(cv::Mat image);

void start_vision_thread_rgb(RemoteAPIObject::sim& sim, int visionSensorHandle);

void start_vision_thread_depth(RemoteAPIObject::sim& sim, int visionSensorHandle);

// 停止视觉显示线程
void stop_vision_thread_rgb();

void stop_vision_thread_depth();

// 检查视觉线程是否在运行
bool is_vision_thread_rgb_running();

bool is_vision_thread_depth_running();

#endif