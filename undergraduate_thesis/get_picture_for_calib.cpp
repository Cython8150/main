#include "visual_module.h"
#include "find_handle.h"
#include "control.h"
#include "time.h"
#include<sys/stat.h>

int main() {
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    sim.startSimulation();
    sim.setStepping(true);

    // 取相机句柄
    int visionSensorHandle = sim.getObject("/UR5/visionSensor");
    cv::Mat image;

    // 取UR5机械臂句柄
    vector<int> ur5_joints(6);
    ur5_joints = findUR5JointHandle(sim);

    int t = time(nullptr);
    string save_path = "image/999999" + to_string(t);
    if (mkdir(save_path.c_str(), 0777) != 0) {
        if (errno != EEXIST) {
            cerr << "无法创建目录: " << save_path << endl;
            return -1;
        }
    }  

    // while(1) {
    //     image = get_rgb_picture(sim, visionSensorHandle);

    //     cv::imshow("eye", image);
    //     int key = cv::waitKey(1) & 0xFF;
    //     if(key == 'q' || key == 27) {
    //         break;
    //     }
    // }

    int count = -1;
    // for(double i = -1.57; i <= 1.57; i+=0.0628) {
    //     image = get_rgb_picture(sim, visionSensorHandle);

    //     cv::imwrite(save_path + "/888888" + to_string(i) + ".jpg", image);
    //     ur5_control(sim, ur5_joints, {i, 0.0, 0.0, 0.0, 0.0, 0.0});

    //     sim.step();
    // }

    double ii = -1.57;
    bool pos = false;
    while(count <=3 ) {
        image = get_rgb_picture(sim, visionSensorHandle);

        //cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

        cv::imwrite(save_path + "/999999" + " " + to_string(ii) + ".jpg", image);
        ur5_control(sim, ur5_joints, {ii, 0.0, 0.0, 0.0, 0.0, 0.0});

        sim.step();
        if(ii == 1.57) {
            pos = true;
            count++;
        }
        else if(ii == -1.57) {
            pos = false;
            count++;
        }

        if(pos==true) ii-=0.0314;
        else ii+=0.0314;
    }

    sim.stopSimulation();
}