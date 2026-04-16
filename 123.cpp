#include <opencv2/opencv.hpp>
#include "RemoteAPIClient.h"
#include "find_handle.h"
#include "visual_module.h"
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <future>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>

using namespace std;
using namespace cv;

// 全局变量
RemoteAPIClient* g_client = nullptr;
RemoteAPIObject::sim* g_sim = nullptr;
vector<int> g_ur5_joints;
bool g_running = true;
vector<double> g_joint_targets(6, 0.0);
vector<bool> g_joint_changed(6, false);
int g_vision_sensor_handle = -1;
atomic<int> g_image_counter(0);
string g_save_path;
mutex g_capture_mutex;
bool g_is_capturing = false;

// 函数声明
void createControlWindow();
void captureImageAsync();
void onJoint1Trackbar(int value, void*);
void onJoint2Trackbar(int value, void*);
void onJoint3Trackbar(int value, void*);
void onJoint4Trackbar(int value, void*);
void onJoint5Trackbar(int value, void*);
void onJoint6Trackbar(int value, void*);

// 滑条回调函数实现
void onJoint1Trackbar(int value, void*) {
    double angle = (value - 314.0) / 100.0;
    g_joint_targets[0] = angle;
    g_joint_changed[0] = true;
    cout << "Joint 1 target: " << angle << " rad (" << angle * 180.0 / 3.14159 << "°)" << endl;
}

void onJoint2Trackbar(int value, void*) {
    double angle = (value - 314.0) / 100.0;
    g_joint_targets[1] = angle;
    g_joint_changed[1] = true;
    cout << "Joint 2 target: " << angle << " rad (" << angle * 180.0 / 3.14159 << "°)" << endl;
}

void onJoint3Trackbar(int value, void*) {
    double angle = (value - 314.0) / 100.0;
    g_joint_targets[2] = angle;
    g_joint_changed[2] = true;
    cout << "Joint 3 target: " << angle << " rad (" << angle * 180.0 / 3.14159 << "°)" << endl;
}

void onJoint4Trackbar(int value, void*) {
    double angle = (value - 314.0) / 100.0;
    g_joint_targets[3] = angle;
    g_joint_changed[3] = true;
    cout << "Joint 4 target: " << angle << " rad (" << angle * 180.0 / 3.14159 << "°)" << endl;
}

void onJoint5Trackbar(int value, void*) {
    double angle = (value - 314.0) / 100.0;
    g_joint_targets[4] = angle;
    g_joint_changed[4] = true;
    cout << "Joint 5 target: " << angle << " rad (" << angle * 180.0 / 3.14159 << "°)" << endl;
}

void onJoint6Trackbar(int value, void*) {
    double angle = (value - 314.0) / 100.0;
    g_joint_targets[5] = angle;
    g_joint_changed[5] = true;
    cout << "Joint 6 target: " << angle << " rad (" << angle * 180.0 / 3.14159 << "°)" << endl;
}

// 创建控制窗口
void createControlWindow() {
    namedWindow("UR5 Joint Control", WINDOW_AUTOSIZE);
    
    // 创建6个滑条
    createTrackbar("Joint 1 (Base)", "UR5 Joint Control", nullptr, 628, onJoint1Trackbar);
    createTrackbar("Joint 2 (Shoulder)", "UR5 Joint Control", nullptr, 628, onJoint2Trackbar);
    createTrackbar("Joint 3 (Elbow)", "UR5 Joint Control", nullptr, 628, onJoint3Trackbar);
    createTrackbar("Joint 4 (Wrist1)", "UR5 Joint Control", nullptr, 628, onJoint4Trackbar);
    createTrackbar("Joint 5 (Wrist2)", "UR5 Joint Control", nullptr, 628, onJoint5Trackbar);
    createTrackbar("Joint 6 (Wrist3)", "UR5 Joint Control", nullptr, 628, onJoint6Trackbar);
    
    // 设置初始值
    setTrackbarPos("Joint 1 (Base)", "UR5 Joint Control", 314);
    setTrackbarPos("Joint 2 (Shoulder)", "UR5 Joint Control", 314);
    setTrackbarPos("Joint 3 (Elbow)", "UR5 Joint Control", 314);
    setTrackbarPos("Joint 4 (Wrist1)", "UR5 Joint Control", 314);
    setTrackbarPos("Joint 5 (Wrist2)", "UR5 Joint Control", 314);
    setTrackbarPos("Joint 6 (Wrist3)", "UR5 Joint Control", 314);
}

// 异步拍照函数
void captureImageAsync() {
    if (g_vision_sensor_handle == -1) {
        cerr << "视觉传感器未初始化" << endl;
        return;
    }
    {
        lock_guard<mutex> lock(g_capture_mutex);
        if (g_is_capturing) {
            cout << "已经在拍照中，请稍候..." << endl;
            return;
        }
        g_is_capturing = true;
    }
    thread([&]() {
        try {
            cout << "\n开始拍照..." << endl;
            
            //int ee_handle = g_sim->getObject("/UR5/link7_visible");
            int ee_handle = g_sim->getObject("/UR5/RG2/attachPoint");
            int base_handle = g_sim->getObject("/UR5");  
            vector<double> ee_pose = g_sim->getObjectPose(ee_handle, base_handle);

            // 获取末端位姿
            // try {
            //     // 你可以根据实际模型修改对象路径
            //     ee_pose = g_sim->getObjectPose(Handle); 
            // } catch (const exception& e) {
            //     cerr << "获取末端位姿失败: " << e.what() << endl;
            //     ee_pose = vector<double>(7, 0.0); // 失败时填充0
            // } 

            // 获取图片
            Mat image;
            bool success = false;
            try {
                auto start_time = chrono::steady_clock::now();
                image = get_rgb_picture(*g_sim, g_vision_sensor_handle);
                auto end_time = chrono::steady_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
                if (image.empty()) {
                    cerr << "获取的图片为空" << endl;
                } else if (duration.count() > 5000) {
                    cerr << "获取图片时间过长: " << duration.count() << "ms" << endl;
                } else {
                    success = true;
                    cout << "获取图片成功，耗时: " << duration.count() << "ms" << endl;
                }
            } catch (const exception& e) {
                cerr << "获取图片异常: " << e.what() << endl;
            }

            if (!success || image.empty()) {
                {
                    lock_guard<mutex> lock(g_capture_mutex);
                    g_is_capturing = false;
                }
                return;
            }

            int counter = g_image_counter++;

            // 生成文件名
            ostringstream filename;
            filename << g_save_path << "/img_" << setfill('0') << setw(3) << counter << ".jpg";
            string filepath = filename.str();

            // 保存图片
            bool save_success = imwrite(filepath, image);

            if (save_success) {
                cout << "图片保存成功: " << filepath << endl;

                // 保存末端位姿信息
                ofstream info_file(g_save_path + "/ee_pose.txt", ios::app);
                if (info_file.is_open()) {
                    info_file << "Image " << counter << ":\n";
                    info_file << "  File: " << filepath << "\n";
                    info_file << "  End-effector pose: ";
                    for (size_t i = 0; i < ee_pose.size(); ++i) {
                        info_file << fixed << setprecision(6) << ee_pose[i];
                        if (i < ee_pose.size() - 1) info_file << " ";
                    }
                    info_file << "\n"; // 末端位姿为[x y z qx qy qz qw]
                    info_file << "\n";
                    info_file.close();
                }
            } else {
                cerr << "图片保存失败" << endl;
            }

        } catch (const exception& e) {
            cerr << "拍照线程异常: " << e.what() << endl;
        }
        {
            lock_guard<mutex> lock(g_capture_mutex);
            g_is_capturing = false;
        }
        cout << "拍照完成" << endl;
    }).detach();
}

int main() {
    cout << "=== UR5 关节控制界面 ===" << endl;
    cout << "使用滑条控制6个关节的角度" << endl;
    cout << "滑条范围: -3.14 到 3.14 弧度 (-180° 到 180°)" << endl;
    cout << "按键说明: ESC退出, R重置, H初始位置, V拍照" << endl;
    
    try {
        // 1. 连接CoppeliaSim
        cout << "\n连接CoppeliaSim..." << endl;
        g_client = new RemoteAPIClient();
        g_sim = new RemoteAPIObject::sim(g_client->getObject().sim());
        
        g_sim->startSimulation();
        g_sim->setStepping(true);
        cout << "连接成功" << endl;
        
        //int Handle = g_sim->getObject("/UR5/link/joint/link/joint/link/joint/link/joint/link/joint/link");

        // 2. 获取关节句柄
        cout << "获取关节句柄..." << endl;
        g_ur5_joints = findUR5JointHandle(*g_sim);
        
        if (g_ur5_joints.empty() || g_ur5_joints[0] == -1) {
            cerr << "错误: 未找到UR5关节" << endl;
            return 1;
        }
        
        cout << "关节句柄获取成功" << endl;
        
        // 3. 获取视觉传感器
        cout << "获取视觉传感器..." << endl;
        try {
            // g_vision_sensor_handle = g_sim->getObject("/UR5/RG2/visionSensor");
            g_vision_sensor_handle = g_sim->getObject("/visionSensor");
        } catch (...) {
            try {
                g_vision_sensor_handle = g_sim->getObject("/UR5/visionSensor");
            } catch (...) {
                cerr << "警告: 未找到视觉传感器" << endl;
            }
        }
        
        if (g_vision_sensor_handle != -1) {
            cout << "视觉传感器句柄: " << g_vision_sensor_handle << endl;
            
            // 测试视觉传感器
            cout << "测试视觉传感器..." << endl;
            try {
                Mat test_image = get_rgb_picture(*g_sim, g_vision_sensor_handle);
                if (!test_image.empty()) {
                    cout << "视觉传感器测试成功，图片尺寸: " 
                         << test_image.cols << "x" << test_image.rows << endl;
                } else {
                    cerr << "视觉传感器测试失败，获取的图片为空" << endl;
                }
            } catch (const exception& e) {
                cerr << "视觉传感器测试异常: " << e.what() << endl;
            }
        }
        
        // 4. 创建保存目录
        int t = time(nullptr);
        g_save_path = "image/hand_eye_new_" + to_string(t);
        if (mkdir(g_save_path.c_str(), 0777) != 0 && errno != EEXIST) {
            cerr << "无法创建目录: " << g_save_path << endl;
        } else {
            cout << "图片保存目录: " << g_save_path << endl;
        }
        
        // 5. 创建控制界面
        cout << "\n创建控制窗口..." << endl;
        createControlWindow();
        
        // 6. 主循环
        cout << "\n控制界面已就绪" << endl;
        cout << "拖动滑条控制关节，按V键拍照" << endl;
        
        auto last_update_time = chrono::steady_clock::now();
        int frames_without_update = 0;
        
        while (g_running) {
            // 6.1 控制更新频率
            auto current_time = chrono::steady_clock::now();
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - last_update_time);
            
            if (elapsed.count() >= 20) {  // 大约50Hz
                // 检查是否有关节需要更新
                bool any_changed = false;
                for (int i = 0; i < 6; i++) {
                    if (g_joint_changed[i]) {
                        try {
                            g_sim->setJointTargetPosition(g_ur5_joints[i], g_joint_targets[i]);
                            g_joint_changed[i] = false;
                            any_changed = true;
                        } catch (const exception& e) {
                            cerr << "设置关节" << (i+1) << "失败: " << e.what() << endl;
                        }
                    }
                }
                
                if (any_changed) {
                    try {
                        g_sim->step();
                        frames_without_update = 0;
                    } catch (const exception& e) {
                        cerr << "仿真步进失败: " << e.what() << endl;
                        frames_without_update++;
                        
                        // 如果连续多次失败，可能是连接问题
                        if (frames_without_update > 10) {
                            cerr << "多次仿真步进失败，可能连接已断开" << endl;
                            break;
                        }
                    }
                }
                
                last_update_time = current_time;
            }
            
            // 6.2 显示界面
            Mat display(400, 600, CV_8UC3, Scalar(50, 50, 50));
            
            // 绘制标题
            putText(display, "UR5 Joint Control Panel", Point(20, 30), 
                   FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            
            // 显示当前关节角度
            for (int i = 0; i < 6; i++) {
                double current_angle = g_joint_targets[i];
                double current_deg = current_angle * 180.0 / 3.14159;
                
                string joint_text = "Joint " + to_string(i+1) + ": " 
                                  + to_string(current_angle).substr(0, 6) + " rad ("
                                  + to_string(current_deg).substr(0, 6) + "°)";
                
                putText(display, joint_text, Point(20, 70 + i*30), 
                       FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
            }
            
            // 显示采集计数
            string count_text = "Images saved: " + to_string(g_image_counter);
            putText(display, count_text, Point(20, 250), 
                   FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
            
            // 显示拍照状态
            {
                lock_guard<mutex> lock(g_capture_mutex);
                if (g_is_capturing) {
                    putText(display, "Status: Capturing...", Point(20, 280), 
                           FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 200, 255), 2);
                } else {
                    putText(display, "Status: Ready", Point(20, 280), 
                           FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
                }
            }
            
            // 显示控制提示
            putText(display, "ESC: Exit  |  R: Reset joints  |  H: Home", Point(20, 310), 
                   FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
            putText(display, "V: Capture image", Point(20, 340), 
                   FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
            
            imshow("UR5 Joint Control", display);
            
            // 6.3 按键检测
            int key = waitKey(1) & 0xFF;
            
            switch (key) {
                case 27:  // ESC键
                    cout << "\n用户退出" << endl;
                    g_running = false;
                    break;
                    
                case 'r':
                case 'R': {
                    // 按R键重置所有关节为0
                    cout << "\n重置所有关节到0度" << endl;
                    for (int i = 0; i < 6; i++) {
                        setTrackbarPos("Joint " + to_string(i+1) + " (Base)", "UR5 Joint Control", 314);
                        g_joint_targets[i] = 0.0;
                        g_joint_changed[i] = true;
                    }
                    break;
                }
                    
                case 'h':
                case 'H': {
                    // 按H键回到初始位置
                    cout << "\n回到初始位置" << endl;
                    vector<int> home_positions = {314, 314, 314, 314, 314, 314};
                    for (int i = 0; i < 6; i++) {
                        setTrackbarPos("Joint " + to_string(i+1) + " (Base)", "UR5 Joint Control", home_positions[i]);
                        g_joint_targets[i] = (home_positions[i] - 314.0) / 100.0;
                        g_joint_changed[i] = true;
                    }
                    break;
                }
                    
                case 'v':
                case 'V': {
                    captureImageAsync();
                    break;
                }
            }
        }
        
        // 7. 清理
        cout << "\n清理资源..." << endl;
        destroyAllWindows();
        
        // 等待所有拍照线程完成
        cout << "等待拍照线程完成..." << endl;
        for (int i = 0; i < 100 && g_is_capturing; i++) {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        
        // 回到初始位置
        cout << "返回初始位置..." << endl;
        for (int i = 0; i < 6; i++) {
            g_sim->setJointTargetPosition(g_ur5_joints[i], 0.0);
        }
        
        for (int i = 0; i < 50; i++) {
            g_sim->step();
            this_thread::sleep_for(chrono::milliseconds(20));
        }
        
        g_sim->stopSimulation();
        
        delete g_sim;
        delete g_client;
        
        cout << "\n=== 程序结束 ===" << endl;
        cout << "总采集图片数: " << g_image_counter << endl;
        cout << "图片保存目录: " << g_save_path << endl;
        cout << "关节角度信息: " << g_save_path << "/joint_angles.txt" << endl;
        
    } catch (const exception& e) {
        cerr << "\n错误: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}