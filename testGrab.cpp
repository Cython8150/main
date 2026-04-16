#include "find_handle.h"
#include "control.h"
#include "visual_module.h"
#include "algorithm"

using namespace std;

int main() {
    // 创建客户端
    RemoteAPIClient client;
    
    // 获取仿真对象
    auto sim = client.getObject().sim();

    // 开始仿真
    sim.startSimulation();

    // 设置步进模式，以便精确控制
    sim.setStepping(true);
    // 1. 获取 UR5 关节句柄
    vector<int> ur5_joints(6);
    ur5_joints = findUR5JointHandle(sim);

    vector<int> rg2_joints(2);  //rg2机械爪的打开关节和关闭关节
    rg2_joints = findRG2JointHandle(sim);

    // 显示视觉传感器图像
    cv::Mat image;
    RemoteAPIClient client_vision_rgb;
    RemoteAPIClient client_vision_depth;
    auto sim_vision_rgb = client_vision_rgb.getObject().sim();
    auto sim_vision_depth = client_vision_depth.getObject().sim();
    //int visionSensorHandle = sim_vision_rgb.getObject("/UR5/RG2/visionSensor");
    int visionSensorHandle = sim_vision_rgb.getObject("/visionSensor");
    int baseHandle = sim.getObject("/UR5");
    start_vision_thread_rgb(sim_vision_rgb, visionSensorHandle);
    start_vision_thread_depth(sim_vision_depth, visionSensorHandle);

    // 张开机械爪
    rg2_control(sim, rg2_joints, 20, 0.05);

    // 执行几步仿真
    for (int i = 0; i < 50; i++) {
        sim.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 4. 移动机械臂到拾取位置
    // vector<double> pick_positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
    // //vector<double> pick_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // ur5_control(sim, ur5_joints, pick_positions);
    
    // 等待机械臂移动到目标位置
    for (int i = 0; i < 50; i++) {
        sim.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 5. 闭合机械爪
    rg2_control(sim, rg2_joints, 20, -0.05);

    cv::Mat image_detect = get_share_rgb_picture();
    vector<float> shared_depth_buffer = get_sha();
    cv::Point3f center_point = detectObject3D(image_detect);
    cout << "center point is " << center_point << endl;
    center_point = point_cam2base(center_point);
    cout << "center point in base is " << center_point << endl;
    int point_fy = 511 - center_point.y;
    cv::circle(image_detect, cv::Point(center_point.x, point_fy), 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(image_detect, cv::Point(center_point.x, center_point.y), 3, cv::Scalar(0, 255, 255), -1);
    cv::imwrite("./1.jpg", image_detect);

    int shapeHandle = sim.getObject("/Cuboid");
    vector<double> objectPose = sim.getObjectPose(shapeHandle, visionSensorHandle);
    for(int i = 0; i < 3; i++) {
        cout << objectPose[i] << " ";
    }
    cout << endl;
    vector<double> objectPoseBase = sim.getObjectPose(shapeHandle, baseHandle);
    for(int i = 0; i < 3; i++) {
        cout << objectPoseBase[i] << " ";
    }
    cout << endl;
    
    // 6. 抬起机械臂
    // std::vector<double> lift_positions = {0.0, -1.0, 1.0, 0.0, 0.0, 0.0};
    // //std::vector<double> lift_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // ur5_control(sim, ur5_joints, lift_positions);
    
    // for (int i = 0; i < 50; i++) {
    //     sim.step();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // }
    
    image_detect = get_share_rgb_picture();
    center_point = detectObject3D(image_detect);
    cout << "center point is " << center_point << endl;
    center_point = point_cam2base(center_point);
    cout << "center point in base is " << center_point << endl;
    point_fy = 511 - center_point.y;
    cv::circle(image_detect, cv::Point(center_point.x, center_point.y), 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(image_detect, cv::Point(center_point.x, point_fy), 3, cv::Scalar(0, 255, 255), -1);
    cv::imwrite("./2.jpg", image_detect);
    //cout << "i: " << in << "   " << "j: " << jn << endl;
    objectPose = sim.getObjectPose(shapeHandle, visionSensorHandle);
    for(int i = 0; i < 3; i++) {
        cout << objectPose[i] << " ";
    }
    cout << endl;
    objectPoseBase = sim.getObjectPose(shapeHandle, baseHandle);
    for(int i = 0; i < 3; i++) {
        cout << objectPoseBase[i] << " ";
    }
    cout << endl;

    // 7. 返回初始位置
    std::vector<double> home_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ur5_control(sim, ur5_joints, home_positions);
    rg2_control(sim, rg2_joints, 20, 0.05);
    
    for (int i = 0; i < 50; i++) {
        sim.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 8. 停止仿真
    printf("Simulation completed!\n");
    stop_vision_thread_rgb();   //停止仿真前，先终止视觉模块的线程
    stop_vision_thread_depth();
    sim.stopSimulation();
    sim_vision_rgb.stopSimulation();
    sim_vision_depth.stopSimulation();

    return 0;
}