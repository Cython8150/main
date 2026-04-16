#include "visual_module.h"
#include <mutex>
#include <condition_variable>

cv::Mat shared_rgb_image;
vector<int> shared_resolution = {512, 512};
vector<float> shared_depth_buffer(262144);
float shared_depth_data = -1.0;

// 线程控制参数
mutex visionSensorMutex;
condition_variable rgb_ready, depth_ready;    // 条件变量
bool rgb_data_ready = false, depth_data_ready = false, rgb_requested = false, depth_requested = false;

vector<float> get_sha() {
    return shared_depth_buffer;
}

// 获取RGB图像
cv::Mat get_rgb_picture(RemoteAPIObject::sim& sim, int visionSensorHandle) {
    unique_lock<mutex> lock(visionSensorMutex);
    if(depth_requested) depth_ready.wait(lock, []{return !depth_requested;});   //等待深度图像获取完毕
    rgb_requested = true;   // RGB线程使用视觉传感器

    auto [img, res] = sim.getVisionSensorImg(visionSensorHandle);

    cv::Mat image = cv::Mat(res[1], res[0], CV_8UC3, img.data());
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    cv::flip(image, image, 0);

    // 重置变量，并通知其他线程
    rgb_requested = false;
    rgb_data_ready = true;
    rgb_ready.notify_all();

    return image;
}

// 获取视觉传感器的RGB图像缓存
cv::Mat get_share_rgb_picture() {
    if(!shared_rgb_image.empty()) return shared_rgb_image;
    else return cv::Mat();
}

// 获取深度图像
cv::Mat get_depth_picture(RemoteAPIObject::sim& sim, int visionSensorHandle) {
    unique_lock<mutex> lock(visionSensorMutex);
    if(rgb_requested) rgb_ready.wait(lock, []{return !rgb_requested;});  // 等待RGB图像获取完毕
    depth_requested = true; //depth线程使用视觉传感器

    auto [depth_buffer, resolution] = sim.getVisionSensorDepth(visionSensorHandle, 1);

    shared_depth_buffer.assign(depth_buffer.begin(), depth_buffer.end());
    if(resolution.size() >= 2) {
        shared_resolution[0] = resolution[0];
        shared_resolution[1] = resolution[1];
    }
    
    // 将字节数组复制到浮点数数组
    memcpy(shared_depth_buffer.data(), depth_buffer.data(), depth_buffer.size());

    // 归一化
    cv::Mat image = cv::Mat(resolution[1], resolution[0], CV_32F, depth_buffer.data());
    cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::flip(image, image, 0);

    // 重置变量，并通知其他线程
    depth_requested = false;
    depth_data_ready = true;
    depth_ready.notify_all();

    return image;
}

// 加载手眼标定结果
cv::Mat loadHandEyeCalibration() {
    cv::FileStorage fs("calibration_results/hand_eye_result.xml", cv::FileStorage::READ);
    cv::Mat T_cam2gripper;
    fs["T_cam2gripper"] >> T_cam2gripper;
    fs.release();
    return T_cam2gripper;
}

cv::Point3f detectObject3D(cv::Mat image) {
    cv::FileStorage fs_camera("./calibration_results/cameraParaments.xml", cv::FileStorage::READ);
    cv::Mat camera_matrix, dist_coeffs;
    fs_camera["intrinsic_matrix"] >> camera_matrix;
    fs_camera["distortion_matrix"] >> dist_coeffs;
    fs_camera.release();

    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Scalar lower_hsv_green(47, 0, 0);
    cv::Scalar upper_hsv_green(155, 255, 255);

    cv::Mat hsv_mask;
    cv::inRange(hsv_image, lower_hsv_green, upper_hsv_green, hsv_mask);

    cv::Mat color_green_mask;
    cv::medianBlur(hsv_mask, color_green_mask, 5);

    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;

    cv::findContours(color_green_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    // 找到最大的轮廓
    if (contours.empty()) {
        return cv::Point3f(-1, -1, -1);
    }
    
    int largest_contour_idx = -1;
    double max_area = 0.0;
    
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            largest_contour_idx = i;
        }
    }
    
    if (largest_contour_idx == -1 || max_area < 100.0) {
        return cv::Point3f(-1, -1, -1);  // 面积太小，忽略
    }
    
    // 使用矩心法计算中心点
    cv::Moments M = cv::moments(contours[largest_contour_idx]);

    if(M.m00 == 0) {
        return cv::Point3f(-1, -1, -1);
    }
    
    int center_x = static_cast<int>(M.m10 / M.m00);
    int center_y = static_cast<int>(M.m01 / M.m00);

    unique_lock<mutex> lock(visionSensorMutex);
    depth_ready.wait(lock, []{return depth_data_ready;});

    int index = (511 - center_y) * 512 + center_x;
    //int index = 118 * shared_resolution[0] + 32;
    float depth_val = shared_depth_buffer[index];
    shared_depth_data = depth_val;

    float fx = camera_matrix.at<double>(0, 0);
    float fy = camera_matrix.at<double>(1, 1);
    float cx = camera_matrix.at<double>(0, 2);
    float cy = camera_matrix.at<double>(1, 2);
    
    //float cout_y = 511 - center_y;

    float X = -(center_x - cx) * depth_val / fx;
    float Y = -(center_y - cy) * depth_val / fy;
    float Z = depth_val;
    
    return cv::Point3f(X, Y, Z);
}

// 显示RGB图像
void vision_display_thread_rgb(RemoteAPIObject::sim& sim, int visionSensorHandle) {
    vision_thread_rgb_running = true;
    
    while(!vision_thread_rgb_stop.load()) {
        cv::Mat frame = get_rgb_picture(sim, visionSensorHandle);   //获取RGB图像
        if(!frame.empty()) {
            shared_rgb_image = frame.clone();
            cv::imshow("rgb picture", shared_rgb_image);
            int key = cv::waitKey(1) & 0xFF;
            if(key == 'q' || key == 27) {
                // 按q通知主线程
                vision_thread_rgb_stop = true;
                break;
            }
        }
    }
    cv::destroyWindow("rgb picture");
    //cv::destroyAllWindows();
    vision_thread_rgb_running = false;
}

// 显示深度图像
void vision_display_thread_depth(RemoteAPIObject::sim& sim, int visionSensorHandle) {
    vision_thread_depth_running = true;

    while(!vision_thread_depth_stop.load()) {
        cv::Mat frame = get_depth_picture(sim, visionSensorHandle); //获取深度图像
        if(!frame.empty()) {

            cv::imshow("depth picture", frame);
            int key = cv::waitKey(1) & 0xFF;
            if(key == 'q' || key == 27) {
                //按q通知主线程
                vision_thread_depth_stop = true;
                break;
            }
        }
    }
    //cv::destroyAllWindows();
    cv::destroyWindow("depth picture");
    vision_thread_depth_running = false;
}

// 开启RGB线程
void start_vision_thread_rgb(RemoteAPIObject::sim& sim, int visionSensorHandle) {
    if(vision_thread_rgb_running.load()) {
        printf("vision_thread_rgb has been started\n");
        return;
    }

    vision_thread_rgb_stop = false;
    vision_thread_rgb = thread(vision_display_thread_rgb, ref(sim), visionSensorHandle);
    
    // 等待线程启动
    this_thread::sleep_for(chrono::milliseconds(100));
}

// 开启深度线程
void start_vision_thread_depth(RemoteAPIObject::sim& sim, int visionSensorHandle) {
    if(vision_thread_depth_running.load()) {
        printf("vision_thread_depth has been started\n");
        return;
    }

    vision_thread_depth_stop = false;
    vision_thread_depth = thread(vision_display_thread_depth, ref(sim), visionSensorHandle);

    // 等待线程启动
    this_thread::sleep_for(chrono::milliseconds(100));
}

void stop_vision_thread_rgb() {
    vision_thread_rgb_stop = true;

    if(vision_thread_rgb.joinable()) vision_thread_rgb.join();
}

void stop_vision_thread_depth() {
    vision_thread_depth_stop = true;

    if(vision_thread_depth.joinable()) vision_thread_depth.join();
}

bool is_vision_thread_rgb_running() {
    return vision_thread_rgb_running.load();
}

bool is_vision_thread_depth_running() {
    return vision_thread_depth_running.load();
}