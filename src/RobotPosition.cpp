#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>

// Global variables
int Venc[2] = {0, 0};  // Kecepatan encoder
float r_wheel = 0.05;  // Radius roda (meter)
float cal_xdot = 0.01, cal_ydot = 0.01;  // Faktor kalibrasi
float imu_angle = 0.0; // Sudut IMU (radian)
float xpos = 0.0, ypos = 0.0, zpos = 0.0; // Posisi robot
std_msgs::Int32MultiArray Encoder_Velocity;

// Konversi derajat ke radian
float d2r(float degree) {
    return degree * M_PI / 180.0;
}

// Callback untuk data encoder velocity
void encoderVelocityCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    Encoder_Velocity = *msg;
}

// Callback untuk data IMU angle
void imuAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
    imu_angle = msg->data;
}

int main(int argc, char** argv) {
    // Inisialisasi ROS node
    ros::init(argc, argv, "RobotPositionNode");
    ros::NodeHandle nh;

    // Publisher & Subscriber
    ros::Publisher position_pub = nh.advertise<geometry_msgs::Vector3>("/robot_position", 10);
    ros::Subscriber encoder_velocity_sub = nh.subscribe("/encoder_velocity", 10, encoderVelocityCallback);
    ros::Subscriber imu_angle_sub = nh.subscribe("/imu_angle", 10, imuAngleCallback);

    // Pesan posisi robot
    geometry_msgs::Vector3 robot_position;

    // Loop rate 100 Hz
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        // Pastikan data encoder tersedia
        if (Encoder_Velocity.data.size() >= 2) {
            Venc[0] = Encoder_Velocity.data[0];
            Venc[1] = Encoder_Velocity.data[1];
        } else {
            ROS_WARN("Encoder velocity array does not have enough elements.");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Perhitungan xdot dan ydot berdasarkan rumus sin dan cos
        float xdot = 0, ydot = 0;
        double dt = 0.01; // Waktu sampling 100 Hz
        float imu_rad = d2r(imu_angle);
        
        xdot = r_wheel * (
            (cos(imu_rad) * (
                cos(d2r(45.)) * Venc[0] +
                cos(d2r(135.)) * Venc[1] 
            )) +
            (sin(imu_rad) * (
                sin(d2r(45)) * Venc[0] +
                sin(d2r(135)) * Venc[1] 
            ))
        );

        ydot = r_wheel * (
            (-sin(imu_rad) * (
                cos(d2r(45.)) * Venc[0] +
                cos(d2r(135.)) * Venc[1] 
            )) +
            (cos(imu_rad) * (
                sin(d2r(45)) * Venc[0] +
                sin(d2r(135)) * Venc[1] 
            ))
        );

 
        // Update posisi
        xpos += xdot * dt;
        ypos += ydot * dt;
        zpos = imu_angle;

        // Perbarui pesan posisi
        robot_position.x = xpos;
        robot_position.y = ypos;
        robot_position.z = zpos;

        // Publish posisi robot
        position_pub.publish(robot_position);


        // ROS loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
