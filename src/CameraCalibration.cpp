#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <regex>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <unistd.h>

// Fungsi untuk menemukan indeks kamera yang tersedia
std::vector<int> findCameraIndices(int max_test = 5) {
    std::vector<int> available_indices;
    for (int i = 0; i < max_test; ++i) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            std::cout << "Camera found at index " << i << std::endl;
            available_indices.push_back(i);
            cap.release();
        } else {
            std::cout << "No camera at index " << i << std::endl;
        }
    }
    return available_indices;
}

// Fungsi untuk mendapatkan serial number kamera menggunakan v4l2-ctl
std::string getCameraSerialNumber(const std::string& device = "/dev/video0") {
    std::string command = "v4l2-ctl -d " + device + " --info";
    std::array<char, 128> buffer;
    std::string result;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return "Unknown";

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    pclose(pipe);

    std::regex serial_regex("Serial Number\\s+:\\s+(\\d+)");
    std::smatch match;
    if (std::regex_search(result, match, serial_regex)) {
        return match[1].str();
    } else {
        return "Unknown";
    }
}

// Fungsi untuk membedakan kamera berdasarkan serial number atau device path
std::map<int, std::string> differentiateCameras(const std::vector<cv::VideoCapture>& cameras, const std::vector<int>& camera_indices) {
    std::map<int, std::string> camera_labels;
    for (size_t i = 0; i < cameras.size(); ++i) {
        std::string device_path = "/dev/video" + std::to_string(camera_indices[i]);
        std::string serial_number = getCameraSerialNumber(device_path);

        if (serial_number == "Unknown") {
            serial_number = device_path;
        }

        camera_labels[i] = serial_number;
        std::cout << "Camera " << i << " Serial Number or Device Path: " << serial_number << std::endl;
    }
    return camera_labels;
}

int main(int argc, char** argv) {
    // Inisialisasi node ROS
    ros::init(argc, argv, "camera_fps_publisher");
    ros::NodeHandle nh;

    // Publisher untuk mempublikasikan FPS di topik /fps_camera
    ros::Publisher fps_pub = nh.advertise<std_msgs::Int32MultiArray>("/fps_camera", 10);

    // Temukan kamera yang tersedia
    std::vector<int> camera_indices = findCameraIndices();
    std::cout << "Available camera indices: ";
    for (int index : camera_indices) {
        std::cout << index << " ";
    }
    std::cout << std::endl;

    // Buka kamera berdasarkan indeks yang terdeteksi
    std::vector<cv::VideoCapture> cameras;
    for (int index : camera_indices) {
        cameras.emplace_back(index);
    }

    // Bedakan kamera berdasarkan serial number atau device path
    std::map<int, std::string> camera_labels = differentiateCameras(cameras, camera_indices);

    // Simpan informasi kamera ke file JSON
    std::ofstream json_file("camera_info.json");
    json_file << "{\n";
    for (auto it = camera_labels.begin(); it != camera_labels.end(); ++it) {
        json_file << "  \"Camera " << it->first << "\": \"" << it->second << "\"";
        if (std::next(it) != camera_labels.end()) {
            json_file << ",";
        }
        json_file << "\n";
    }
    json_file << "}\n";
    json_file.close();

    // Inisialisasi untuk menghitung FPS
    std::vector<int> fps_counters(cameras.size(), 0);
    std::vector<ros::Time> fps_start_times(cameras.size(), ros::Time::now());

    ros::Rate loop_rate(30); // Loop rate untuk ROS

    while (ros::ok()) {
        std_msgs::Int32MultiArray fps_msg;
        fps_msg.data.resize(cameras.size());

        for (size_t i = 0; i < cameras.size(); ++i) {
            cv::Mat frame;
            if (cameras[i].read(frame)) {
                // Hitung FPS
                fps_counters[i]++;
                ros::Duration elapsed_time = ros::Time::now() - fps_start_times[i];
                double seconds = elapsed_time.toSec();

                if (seconds >= 1.0) {
                    double fps = fps_counters[i] / seconds;
                    fps_msg.data[i] = static_cast<int>(fps); // Tambahkan FPS ke pesan

                    std::cout << "Camera " << i << " (" << camera_labels[i] << "): " << fps << " FPS" << std::endl;

                    // Reset counter dan waktu
                    fps_counters[i] = 0;
                    fps_start_times[i] = ros::Time::now();
                }
            }
        }

        // Publikasikan data FPS ke topik /fps_camera
        fps_pub.publish(fps_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Lepaskan semua kamera
    for (auto& cam : cameras) {
        cam.release();
    }

    return 0;
}
