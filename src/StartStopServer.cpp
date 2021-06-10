#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ros/time.h"

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
    RECHTS_GERADEAUS_KURZ,
    GERADEAUS_LANG,
    HUNDERTACHTZIG,
    NEUNZIG_LINKS,
    NEUNZIG_RECHTS


} ROBOT_MOVEMENT;



bool robot_move(const ROBOT_MOVEMENT move_type) {
    if (move_type == STOP) {
        ROS_INFO("STOP! \n");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
        motor_command_publisher.publish(motor_command);
    } else if (move_type == GERADEAUS_LANG) {
        ROS_INFO("Geradeaus Lang! \n");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.05;
        motor_command_publisher.publish(motor_command);
    } else if (move_type == RECHTS_GERADEAUS_KURZ) {
        ROS_INFO("Wenden! \n");
        float angular_speed = -1.0;
        float rate = 50.0;
        float goal_angle = 3.1415927/2.0f;
        //float angular_duration = goal_angle/angular_speed;
        int ticks = int(goal_angle * rate);
        for(int i = 0; i<=ticks;i++){
            ROS_INFO("Ticks: %i", ticks);
            ROS_INFO("i: %i", i);
            ROS_INFO("Linksdrehung! \n");

            motor_command.angular.z = angular_speed;
            motor_command_publisher.publish(motor_command);
            ros::Duration(0, 21000000).sleep();
        }
        ROS_INFO("Geradeaus Kurz! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = 0.0;
        motor_command_publisher.publish(motor_command);
        ros::Duration(0, 21000000).sleep();

    } else if (move_type == HUNDERTACHTZIG) {
        ROS_INFO("Wenden! \n");
        float angular_speed = 1.0;
        float rate = 50.0;
        float goal_angle = 3.1415927;
        //float angular_duration = goal_angle/angular_speed;
        int ticks = int(goal_angle * rate);
        for(int i = 0; i<=ticks;i++){
            ROS_INFO("Ticks: %i", ticks);
            ROS_INFO("i: %i", i);
            ROS_INFO("Wenden! \n");

            motor_command.angular.z = angular_speed;
            motor_command_publisher.publish(motor_command);
            ros::Duration(0, 21000000).sleep();
        }
        motor_command.angular.z = 0.0;
        motor_command_publisher.publish(motor_command);
    } else if (move_type == NEUNZIG_LINKS) {
        ROS_INFO("Neunzig Links! \n");
        float angular_speed = 1.0;
        float rate = 50.0;
        float goal_angle = 3.1415927/2.0f;
        //float angular_duration = goal_angle/angular_speed;
        int ticks = int(goal_angle * rate);
        for(int i = 0; i<=ticks;i++){
            ROS_INFO("Ticks: %i", ticks);
            ROS_INFO("i: %i", i);
            ROS_INFO("Links! \n");

            motor_command.angular.z = angular_speed;
            motor_command_publisher.publish(motor_command);
            ros::Duration(0, 21000000).sleep();
        }
    } else if (move_type == NEUNZIG_RECHTS) {

    }else {
        ROS_INFO("Move type wrong! \n");
        return false;
    }

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
                    robot_move(HUNDERTACHTZIG);                                //180° Linksdrehung, da Wand links vorhanden ist
                    counter = 1;                    //um in if-Bedingung (counter ==1) zu kommen
                } else{
                    robot_move(NEUNZIG_LINKS);          //90° Linksdrehung
                    counter = 1;
                }
            } else{
                robot_move(GERADEAUS_LANG);                              //Turtlebot fährt dicht zur ersten Wand ab der er sich orientieren kann
            }
        } else{                             //ab hier an kann der Turtlebot sich orientieren
           if(averageRechts <= 0.2){            //Turtlebot hat rechts neben sich eine Wand und kann somit den Rechte-Hand Algorythmus durchführen
               if(averageVorne <= 0.2){             //Turtlebot hat eine Wand vor sich und eine Wand rechts neben sich
                   if(averageLinks <= 0.2){             //Kommentar aus Zeile 153 + noch eine Wand dicht auf der Linken Seite
                       robot_move(HUNDERTACHTZIG);                                    //180° Linksdrehung da der Turtlebot in einer Sackgasse ist
                   } else{                              //Links hat der Turtlebot keine Wand
                       robot_move(NEUNZIG_LINKS);                                 //Linksdrehung um 90°
                   }
               } else{                              //Turtlebot hat keine Wand vor sich aber rechts neben sich, daher kann er geradeaus fahren
                   robot_move(GERADEAUS_LANG);
               }
           } else {                             //Turtlebot hat keine Wand rechts neben sich daher ein Gang oder eine Tür
               robot_move(RECHTS_GERADEAUS_KURZ);                                   //Turtlebot dreht sich 90° nach rechts und bewegt sich ein Stück nach vorne
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

    /*ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

     */
    ros::Rate r(50.0);



    ros::Duration time_between_ros_wakeups(0.001);



    while (ros::ok()) {
        geometry_msgs::Twist msg;
        ros::Time::init();




        /*current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();*/

        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    return 0;

}