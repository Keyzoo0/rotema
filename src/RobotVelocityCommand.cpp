#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Quaternion.h>


#define lamdaFar 2
#define lamdaNear 1



class MotorVelocityNode {
private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber goal_position_sub;
    ros::Subscriber robot_position_sub_;

    // Publisher
    ros::Publisher motor_velocity_pub_;

    // Data storage
    geometry_msgs::Vector3 goal_position;
    geometry_msgs::Vector3 robot_position_;
    bool nav_goal_received_ = false;
    bool robot_position_received_ = false;
    float r_wheel = 0.05;
    float imu_angle = 0;
    float axle_lenght = 5; //25,5 cm 


public:
    float d2r(int degree){
        float radians = degree * 0.0174533;
        return radians;
    }

    MotorVelocityNode() {
        // Initialize subscribers
        goal_position_sub = nh_.subscribe("/goal_position", 10, &MotorVelocityNode::goalPositionCallback, this);
        robot_position_sub_ = nh_.subscribe("/robot_position", 10, &MotorVelocityNode::robotPositionCallback, this);

        // Initialize publisher
        motor_velocity_pub_ = nh_.advertise<geometry_msgs::Quaternion>("/motor_velocity_command", 10);
    }

    void goalPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        goal_position = *msg;
        nav_goal_received_ = true;
        computeAndPublishVelocity();
    }

    void robotPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        robot_position_ = *msg;
        robot_position_received_ = true;
        computeAndPublishVelocity();
    }

    void computeAndPublishVelocity() {
        if (!nav_goal_received_ || !robot_position_received_) {
            return; // Wait until both topics have been received
        }

        imu_angle = robot_position_.z;
        // Compute the velocity vector (difference between goal and position)
        geometry_msgs::Vector3 error;
        error.x = robot_position_.x -  goal_position.x ;
        error.y = robot_position_.y - goal_position.y ;
        error.z = robot_position_.z -  goal_position.z ;
        float gain = 0;
        float error_norm1 = sqrt((pow(error.x,2))
                            + pow(error.y , 2));
        float error_norm2 = sqrt(pow(error.x , 2) + pow(error.y , 2) + pow(error.z , 2)  );

        if(error_norm1 < 0.1){
            gain = 0;
        }
        else if(error_norm1<0.7)gain = lamdaNear;
        else gain = lamdaFar;
        
        geometry_msgs::Quaternion motor_velocity_cmd;

        //motor1
        motor_velocity_cmd.x = (1 / r_wheel) * gain * (
            cos(d2r(45.)) * (cos(imu_angle) * error.x - sin(imu_angle) * error.y) +
            sin(d2r(45.)) * (sin(imu_angle) * error.x + cos(imu_angle) * error.y)
        ) + (axle_lenght * error.z);
        //motor2
        motor_velocity_cmd.y = (1 / r_wheel) * gain * (
            cos(d2r(-45.)) * (cos(imu_angle) * error.x - sin(imu_angle) * error.y) +
            sin(d2r(-45.)) * (sin(imu_angle) * error.x + cos(imu_angle) * error.y)
        ) + (axle_lenght * error.z);
        //motor3
        motor_velocity_cmd.z = (1 / r_wheel) * gain * (
            cos(d2r(-135.)) * (cos(imu_angle) * error.x - sin(imu_angle) * error.y) +
            sin(d2r(-135.)) * (sin(imu_angle) * error.x + cos(imu_angle) * error.y)
        ) + (axle_lenght * error.z);
        //motor4
        motor_velocity_cmd.w = (1 / r_wheel) * gain * (
            cos(d2r(135.)) * (cos(imu_angle) * error.x - sin(imu_angle) * error.y) +
            sin(d2r(135.)) * (sin(imu_angle) * error.x + cos(imu_angle) * error.y)
        ) + (axle_lenght * error.z);
            
        motor_velocity_pub_.publish(motor_velocity_cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_velocity_node");
    MotorVelocityNode motor_velocity_node;
    ros::spin();
    return 0;
}
