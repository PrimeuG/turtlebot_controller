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
static int counter = 0;

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
        motor_command.linear.x = 0.01;
        motor_command.angular.z = -0.15;
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

    float summeVorne = 0.0;
    float summeLinks = 0.0;
    float summeRechts = 0.0;
    float summeVorneRechts = 0.0;

    float averageVorne = 0.0;
    float averageLinks = 0.0;
    float averageRechts = 0.0;
    float averageVorneRechts = 0.0;


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
        if (z == 275) {
            averageRechts = summeRechts / 11.0;
        }
    }

    for (int z = 340; z < 346; z++) {
        summeVorneRechts += laser_ranges[z];
        if (z == 345) {
            averageVorneRechts = summeVorneRechts / 6.0;

        }
    }
    if (averageVorne <= 0 || averageVorne > 3.5 || averageLinks <= 0 || averageLinks > 3.5 || averageRechts <= 0 ||
        averageRechts > 3.5 || averageVorneRechts <= 0 || averageVorneRechts > 3.5) {
        ROS_INFO("STÖRUNG");

    } else {
        ROS_INFO("averageRechts: %f", averageRechts);
        ROS_INFO("averageLinks: %f", averageLinks);
        ROS_INFO("averageVorne: %f", averageVorne);
        ROS_INFO("averageVorneRechts: %f", averageVorneRechts);
        ros::Rate rate(1);

        if (counter == 0){                  //Ist für den Anfang damit der Turtlebot von der mitte aus zur ersten Wand fährt
            if(averageVorne <= 0.2){            //Turtlebot hat Wand vor sich erreicht

                if(averageLinks <= 0.2){            //Um Wand auf der Rechten Seite des Turtlebots zu haben muss er sich Linksdrehen
                                                    //180° Linksdrehung, da Wand links vorhanden ist
                    counter = 1;                    //um in if-Bedingung (counter ==1) zu kommen
                } else{
                                                    //90° Linksdrehung
                    counter = 1;
                }
            } else{
                                                //Turtlebot fährt dicht zur ersten Wand ab der er sich orientieren kann
            }
        } else{                             //ab hier an kann der Turtlebot sich orientieren
           if(averageRechts <= 0.2){            //Turtlebot hat rechts neben sich eine Wand und kann somit den Rechte-Hand Algorythmus durchführen
               if(averageVorne <= 0.2){             //Turtlebot hat eine Wand vor sich und eine Wand rechts neben sich
                   if(averageLinks <= 0.2){             //Kommentar aus Zeile 153 + noch eine Wand dicht auf der Linken Seite
                                                        //180° Linksdrehung da der Turtlebot in einer Sackgasse ist
                   } else{                              //Links hat der Turtlebot keine Wand
                                                        //Linksdrehung um 90°
                   }
               } else{                              //Turtlebot hat keine Wand vor sich aber rechts neben sich, daher kann er geradeaus fahren

               }
           } else {                             //Turtlebot hat keine Wand rechts neben sich daher ein Gang oder eine Tür
                                                //Turtlebot dreht sich 90° nach rechts und bewegt sich ein Stück nach vorne
           }
        }

    }
    /*if (averageRechts > 0.5 && averageLinks > 0.5 && averageVorne >= 0.2 && counter == 0) {
        robot_move(FORWARD);
    } else if (averageRechts > 0.5 && averageLinks > 0.5 && averageVorne <= 0.2 && counter == 0) {
        robot_move(TURN_LEFT);
        rate.sleep();
    } else {
        if (averageRechts > 0.2) {
            robot_move(GO_RIGHT);   //rechts fahren bis zur Wand
            counter = 1;
        } else {
            if (averageVorne > 0.2 ) {
                robot_move(FORWARD);    //da dicht genug an der rechten Wand fahr vorne
                if (averageVorneRechts < 0.15){
                    robot_move(GO_LEFT);
                }
                counter = 1;
            } else if (averageVorne <= 0.20 && averageLinks > 0.20) {
                robot_move(
                        GO_LEFT);    //da Vorne und Rechts dicht an wand , nach links fahren da Laserwerte keine Wand anzeigen
                counter = 1;
            } else if (averageVorne <= 0.20 && averageLinks <= 0.20) {
                robot_move(
                        GO_LEFT);  //da Vorne, Rechts und Links Laserwerte anzeigen das Wände sehr dicht sind, umdrehen
                counter = 1;
            } else {
                ROS_INFO("Was geht hier ab");
            }
        }
    }

}*/





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
    }!*/
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