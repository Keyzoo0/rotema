#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>

class TaskNavigator {
public:
    TaskNavigator() {
        // Publisher untuk /goal_position
        goal_position_pub_ = nh_.advertise<geometry_msgs::Vector3>("/goal_position", 10);

        // Subscriber untuk /task_index
        task_index_sub_ = nh_.subscribe("/task_index", 10, &TaskNavigator::taskIndexCallback, this);
    }

    void taskIndexCallback(const std_msgs::Int32::ConstPtr& msg) {
        int index = (int)msg->data;  // Perbaikan: tidak mengubah index menjadi 0

        // Generate a navigation goal based on the task index
        geometry_msgs::Vector3 goal_position;
        
        switch(index) {
            case 1:
                goal_position.x = 100;
                goal_position.y = 0;
                goal_position.z = -90;
                break;  // Perbaikan: Tambahkan break agar hanya menjalankan case yang sesuai
            default:
                goal_position.x = 0;
                goal_position.y = 0;
                goal_position.z = 0;
                break;
        }

        // Publish the navigation goal
        goal_position_pub_.publish(goal_position);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher goal_position_pub_;
    ros::Subscriber task_index_sub_;
};

int main(int argc, char** argv) {
    // Inisialisasi ROS node
    ros::init(argc, argv, "RobotNavigationNode");

    // Membuat instance TaskNavigator
    TaskNavigator task_navigator;

    // Menjalankan loop ROS
    ros::spin();

    return 0;
}
