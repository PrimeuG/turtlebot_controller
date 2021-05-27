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


bool robot_move(const ROBOT_MOVEMENT move_type) {
    if (move_type == STOP) {
        ROS_INFO("STOP! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    } else if (move_type == FORWARD) {
        ROS_INFO("Geradeaus! \n");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.17;
    } else if (move_type == BACKWARD) {
        ROS_INFO("Rückwaerts! \n");
        motor_command.linear.x = -0.15;
        motor_command.angular.z = 0.75;
    } else if (move_type == TURN_LEFT) {
        ROS_INFO("Linksdrehung! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = 0.5;
    } else if (move_type == TURN_RIGHT) {
        ROS_INFO("Rechtsdrehung! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = -0.5;
    } else if (move_type == GO_RIGHT) {
        ROS_INFO("Rechts! \n");
        motor_command.linear.x = 0.15;
        motor_command.angular.z = -0.5;
    } else if (move_type == GO_LEFT) {
        ROS_INFO("Links! \n");
        motor_command.linear.x = 0.15;
        motor_command.angular.z = 0.5;
    } else {
        ROS_INFO("Move type wrong! \n");
        return false;
    }
    motor_command_publisher.publish(motor_command);
    usleep(10);
    return true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {

    laser_msg = *msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    float range_min = laser_msg.range_max;

    float summeVorne;
    float summeLinks;
    float summeRechts;


    for (int z = 0; z < 11; z++) {
        summeVorne += laser_ranges[z];
    }

    for (int k = 350; k < 360; k++) {
        summeVorne += laser_ranges[k];
    }

    for (int z = 80; z < 101; z++) {
        summeLinks += laser_ranges[z];
    }

    for (int z = 260; z < 281; z++) {
        summeRechts += laser_ranges[z];
    }



    float averageVorne;
    float averageLinks;
    float averageRechts;

    averageVorne = summeVorne/21;
    averageLinks = summeLinks/21;
    averageLinks = summeRechts/21;



   /* for (size_t i = 0; i < range_size; i++) {
        if (laser_ranges[i] < range_min) {
            range_min = laser_ranges[i];
        }*/


        /*for (float x : msg->ranges) {  //geht die Laser Wertw in der For Schleife durch
            if (x >= msg->range_min && x <= msg->range_max && x <=sensorEntfernung) { //wenn der x Wert größergleich ist als die min range und kleinergleich ist als die max range und
                sensorEntfernung = x;     //kleiner gleich der SensorReichweite von 3,5 ist wird die sensorEntfernung auf x gesetzt
            } else {        //ansonsten wird der Wert um 1 erhöht
                counter += 1;
            }
        }
    }*/
    }


    int main(int argc, char **argv) {
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