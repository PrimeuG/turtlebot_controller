#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

using namespace std_msgs;

//https://github.com/cohnsted1/TurtleBot--MazeSolver/blob/main/src/new_main.cpp

ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    GO_RIGHT,
    GO_LEFT

} ROBOT_MOVEMENT;


bool robot_move(const ROBOT_MOVEMENT move_type)
{
    if (move_type == STOP) {
        ROS_INFO("STOP! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    }

    else if (move_type == FORWARD) {
        ROS_INFO("Geradeaus! \n");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.17;
    }

    else if (move_type == BACKWARD) {
        ROS_INFO("Rückwaerts! \n");
        motor_command.linear.x = -0.15;
        motor_command.angular.z = 0.75;
    }

    else if (move_type == TURN_LEFT) {
        ROS_INFO("Linksdrehung! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = 0.5;
    }

    else if (move_type == TURN_RIGHT) {
        ROS_INFO("Rechtsdrehung! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = -0.5;
    }
    else if (move_type == GO_RIGHT) {
        ROS_INFO("Rechts! \n");
        motor_command.linear.x = 0.15;
        motor_command.angular.z = -0.5;
    }
    else if (move_type == GO_LEFT) {
        ROS_INFO("Links! \n");
        motor_command.linear.x = 0.15;
        motor_command.angular.z = 0.5;
    }
    else {
        ROS_INFO("Move type wrong! \n");
        return false;
    }
    motor_command_publisher.publish(motor_command);
    usleep(10);
    return true;
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

}

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "node");
    ros::NodeHandle n;
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    laser_subscriber = n.subscribe("/scan", 1000, laserCallback);
    ros::Duration time_between_ros_wakeups(0.001);

    while (ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    return 0;

}