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
        motor_command.linear.x = 0.05;
    } else if (move_type == BACKWARD) {
        ROS_INFO("Rückwaerts! \n");
        motor_command.linear.x = -0.15;
        motor_command.angular.z = 0.75;
    } else if (move_type == TURN_LEFT) {
        ROS_INFO("Linksdrehung! \n");
        motor_command.linear.x = 0.00;
        motor_command.angular.z = 0.05;
    } else if (move_type == TURN_RIGHT) {
        ROS_INFO("Rechtsdrehung! \n");
        motor_command.linear.x = 0.00;
        motor_command.angular.z = -0.05;
    } else if (move_type == GO_RIGHT) {
        ROS_INFO("Rechts! \n");
        motor_command.linear.x = 0.025;
        motor_command.angular.z = -0.08;
    } else if (move_type == GO_LEFT) {
        ROS_INFO("Links! \n");
        motor_command.linear.x = 0.0;
        motor_command.angular.z = 0.05;
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
    float summeVorneRechts;

    float averageVorne;
    float averageLinks;
    float averageRechts;
    float averageVorneRechts;


    for (int z = 0; z < 6; z++) {
        summeVorne += laser_ranges[z];
    }

    for (int k = 355; k < 360; k++) {
        summeVorne += laser_ranges[k];
        if (k == 359) {
            averageVorne = summeVorne / 11.0;
        }

    }

    for (int z = 85; z < 96; z++) {
        summeLinks += laser_ranges[z];
        if (z == 95) {
            averageLinks = summeLinks / 11.0;
        }

    }

    for (int z = 265; z < 276; z++) {
        summeRechts += laser_ranges[z];
        if (z == 280) {
            averageRechts = summeRechts / 11.0;
        }
    }

    for (int z = 310; z < 321; z++) {
        summeVorneRechts += laser_ranges[z];
        if (z == 320) {
            averageVorneRechts = summeVorneRechts / 11.0;
        }
    }


    ROS_INFO("averageRechts: %f", averageRechts);
    ROS_INFO("averageLinks: %f", averageLinks);
    ROS_INFO("averageVorne: %f", averageVorne);

    if (averageRechts && averageLinks > 0.5 && averageVorne >= 0.3) {
        robot_move(FORWARD);
    } else if (averageRechts && averageLinks > 0.5 && averageVorne <= 0.3) {
        robot_move(TURN_LEFT);
    } else {
        if (averageRechts > 0.35) {
            robot_move(GO_RIGHT);       //rechts fahren bis zur Wand

        } else {
            if (averageVorne > 0.35) {
                robot_move(FORWARD);    //da dicht genug an der rechten Wand fahr vorne
            } else if (averageVorne <= 0.25 && averageLinks > 0.25) {
                robot_move(
                        GO_LEFT);    //da Vorne und Rechts dicht an wand , nach links fahren da Laserwerte keine Wand anzeigen
            } else if (averageVorne <= 0.25 && averageLinks <= 0.25) {
                robot_move(
                        TURN_LEFT);  //da Vorne, Rechts und Links Laserwerte anzeigen das Wände sehr dicht sind, umdrehen
            }
        }
    }





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