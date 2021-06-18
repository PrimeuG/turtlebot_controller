#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ros/time.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

#define RAD2DEG 57.295779513
using namespace std_msgs;

//https://github.com/cohnsted1/TurtleBot--MazeSolver/blob/main/src/new_main.cpp

ros::Publisher motor_command_publisher;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;


static int counter = 1;
float posiZ = 0.0;
static float WirdUmgenannt = 0;
static int Richtungsgeber = 0;
static float averageVorne = 0.0;
static float averageLinks = 0.0;
static float averageRechts = 0.0;
static float averageVorneRechts = 0.0;
static int faelle = 0;
static int test37 = 0;
static int koordinatenVorwaerts = 0;

float previousX = 0; //aktuellste X Position
float rechnerxx = 0; //erster X Wert
float previousy = 0;//aktuellste X Position
float rechneryy = 0; //erster Y Wert
float weg = 0;


// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    GERADEAUS_KURZ,
    GERADEAUS_LANG,
    GERADEAUS_MITTEL,
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
    } else if (move_type == GERADEAUS_KURZ) {
        //ROS_INFO("GERADEAUS_KURZ! \n");

        //float angular_duration = goal_angle/angular_speed;
        motor_command.angular.z = 0.0;
        motor_command_publisher.publish(motor_command);

        //ROS_INFO("GERADEAUS_KURZ! \n");


        for (int k = 0; k < 300000; k++) {
            motor_command.linear.x = 0.1;
            motor_command_publisher.publish(motor_command);
        }
        //motor_command_publisher.publish(motor_command);
        //ros::Duration(0, 15000000).sleep();

    } else if (move_type == GERADEAUS_MITTEL) {

        //ROS_INFO("Geradeaus Mittel! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.05;
        motor_command_publisher.publish(motor_command);

    } else if (move_type == NEUNZIG_LINKS) {

        ROS_INFO("Neunzig Links! \n");
        motor_command.linear.x = 0.0;
        motor_command.angular.z = 0.1;
        motor_command_publisher.publish(motor_command);
    } else if (move_type == NEUNZIG_RECHTS) {

        // ROS_INFO("Neunzig Rechts! \n");
        motor_command.linear.x = 0.0;
        motor_command.angular.z = -0.1;
        motor_command_publisher.publish(motor_command);
    } else {
        ROS_INFO("Move type wrong! \n");
        return false;
    }

    usleep(10);
    return true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("Richtungsgeber %i", Richtungsgeber);
    ros::Rate rateH(5);
    laser_msg = *msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    float range_min = laser_msg.range_max;
    ros::Rate randomer(50.0);

    float summeVorne = 0.0;
    float summeLinks = 0.0;
    float summeRechts = 0.0;
    float summeVorneRechts = 0.0;


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

   // rateH.sleep();
    //ROS_INFO("VORNE %f", averageVorne);
    if (averageVorne <= 0 || averageVorne > 3.5 || averageLinks <= 0 || averageLinks > 3.5 || averageRechts <= 0 ||
        averageRechts > 3.5 || averageVorneRechts <= 0 || averageVorneRechts > 3.5) {
        ROS_INFO("STÖRUNG");
        ROS_INFO("averageRechts: %f", averageRechts);
        ROS_INFO("averageLinks: %f", averageLinks);
        ROS_INFO("averageVorne: %f", averageVorne);
        ROS_INFO("averageVorneRechts: %f", averageVorneRechts);

    } else {
        ROS_INFO("averageRechts: %f", averageRechts);
        ROS_INFO("averageLinks: %f", averageLinks);
        ROS_INFO("averageVorne: %f", averageVorne);
        //ROS_INFO("averageVorneRechts: %f", averageVorneRechts);
        ros::Rate rate(1);

        //ROS_INFO("Counter: %i", counter);


        if (test37 == 1) {

            switch (Richtungsgeber) {
                case 0:

                    //ROS_INFO("HALLO: %f", WirdUmgenannt);
                    while (WirdUmgenannt < 90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    test37 = 0;
                    Richtungsgeber = 90;
                    ros::spinOnce();
                    break;
                case 90:
                    while (WirdUmgenannt < 180 && WirdUmgenannt > 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }
                    Richtungsgeber = 180;
                    test37 = 0;
                    ros::spinOnce();
                    break;
                case 180:
                    if (WirdUmgenannt > 0.0) {
                        while (WirdUmgenannt > 0.0) {
                            robot_move(NEUNZIG_LINKS);
                            ros::spinOnce();
                        }
                    }
                    while (WirdUmgenannt < -90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }
                    Richtungsgeber = -90;
                    test37 = 0;
                    ros::spinOnce();
                    break;
                case -90:
                    while (WirdUmgenannt < 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }
                    Richtungsgeber = 0;
                    test37 = 0;
                    ros::spinOnce();
                    break;
            }

        }


        if (test37 == 2) {
            switch (Richtungsgeber) {
                case 0:
                    //ROS_INFO("HALLO: %f", WirdUmgenannt);
                    while (WirdUmgenannt < 180.0 && WirdUmgenannt > -1.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }
                    Richtungsgeber = 180;
                    test37 = 0;

                    ros::spinOnce();
                    break;
                case 90:
                    while (WirdUmgenannt <= 180.0) {
                        robot_move(NEUNZIG_LINKS);

                    }
                    while (WirdUmgenannt < -90.0) {
                        robot_move(NEUNZIG_LINKS);

                    }
                    Richtungsgeber = -90.0;
                    test37 = 0;
                    ros::spinOnce();
                    break;
                case 180:
                    if (WirdUmgenannt > 0.0) {
                        while (WirdUmgenannt > 0.0) {
                            robot_move(NEUNZIG_LINKS);

                        }
                    }
                    while (WirdUmgenannt < 0.0) {
                        robot_move(NEUNZIG_LINKS);

                    }
                    Richtungsgeber = 0;
                    test37 = 0;
                    ros::spinOnce();
                    break;
                case -90:
                    while (WirdUmgenannt <= 0.0) {
                        robot_move(NEUNZIG_LINKS);

                    }
                    while (WirdUmgenannt < 90.0) {
                        robot_move(NEUNZIG_LINKS);

                    }
                    Richtungsgeber = 90;
                    test37 = 0;
                    ros::spinOnce();
                    break;
            }
        }

        if (test37 == 3) {
            ROS_INFO("Fälle: %i", faelle);
            ROS_INFO("Umgenannt: %f", WirdUmgenannt);
            ROS_INFO("Richtungsgeber: %i", Richtungsgeber);

            for (int y = 0; y < 20; y++) {
                //ROS_INFO("SWITCH CHECKPOINT 1");
            }
            if (faelle == 4) {

                while(weg < 0.12 && weg > -0.12 ){

                    robot_move(GERADEAUS_MITTEL);
                    ros::spinOnce();
                }
                faelle = 1;
                koordinatenVorwaerts = 0;
                ros::spinOnce();

            } else if (faelle == 1) {

                switch (Richtungsgeber) {

                    case 0:
                        while (WirdUmgenannt > -90.0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }
                        faelle = 2;
                        Richtungsgeber = -90;
                        ros::spinOnce();
                        break;
                    case 90:
                        while (WirdUmgenannt > 0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }
                        faelle = 2;
                        Richtungsgeber = 0;
                        ros::spinOnce();
                        break;
                    case 180:
                      //  ROS_INFO("HALLO: %f", WirdUmgenannt);
                        while (WirdUmgenannt > 90 || WirdUmgenannt < 0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }
                        faelle = 2;
                        Richtungsgeber = 90;
                        ros::spinOnce();
                        break;
                    case -90:
                        while (WirdUmgenannt > -179 && WirdUmgenannt < 1) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }
                        faelle = 2;
                        Richtungsgeber = 180;
                        ros::spinOnce();
                        break;
                }

            } else if (faelle == 2) {
                for (int y = 0; y < 20; y++) {
                    ROS_INFO("SWITCH CHECKPOINT 3");

                }

                while(weg < 0.12 && weg > -0.12 ){
                    robot_move(GERADEAUS_MITTEL);
                    ros::spinOnce();
                }

                koordinatenVorwaerts = 0;
                faelle = 0;
                test37 = 0;
                rateH.sleep();
                ros::spinOnce();


            }


        }
        /*if (counter == 0) {//Ist für den Anfang damit der Turtlebot von der mitte aus zur ersten Wand fährt
            ros::spinOnce();
            if (averageVorne <= 0.2) {            //Turtlebot hat Wand vor sich erreicht

                if (averageLinks <=
                    0.2) {            //Um Wand auf der Rechten Seite des Turtlebots zu haben muss er sich Linksdrehen
                    switch (Richtungsgeber) {
                        case 0:
                            while (WirdUmgenannt < 90.0) {
                                robot_move(NEUNZIG_LINKS);
                                test37 = 1;
                                ros::spinOnce();
                            }
                            Richtungsgeber = 90;
                            break;
                        case 90:
                            while (WirdUmgenannt < 180 && WirdUmgenannt > 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 180;
                            break;
                        case 180:
                            if (WirdUmgenannt > 0.0) {
                                while (WirdUmgenannt > 0.0) {
                                    robot_move(NEUNZIG_LINKS);
                                }
                            }
                            while (WirdUmgenannt < -90.0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = -90;
                            break;
                        case -90:
                            while (WirdUmgenannt < 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 0;
                            break;
                    }                               //180° Linksdrehung, da Wand links vorhanden ist
                    counter = 1;                    //um in if-Bedingung (counter ==1) zu kommen
                } else {
                    switch (Richtungsgeber) {
                        case 0:
                            while (WirdUmgenannt < 90.0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 90;
                            break;
                        case 90:
                            while (WirdUmgenannt < 180 && WirdUmgenannt > 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 180;
                            break;
                        case 180:
                            if (WirdUmgenannt > 0.0) {
                                while (WirdUmgenannt > 0.0) {
                                    robot_move(NEUNZIG_LINKS);
                                }
                            }
                            while (WirdUmgenannt < -90.0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = -90;
                            break;
                        case -90:
                            while (WirdUmgenannt < 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 0;
                            break;
                    }                       //90° Linksdrehung
                    counter = 1;
                }
            } else {
                robot_move(
                        GERADEAUS_LANG);                              //Turtlebot fährt dicht zur ersten Wand ab der er sich orientieren kann
            }
        }*/ //if {                             //ab hier an kann der Turtlebot sich orientieren
            if (averageRechts <=
                0.25) {            //Turtlebot hat rechts neben sich eine Wand und kann somit den Rechte-Hand Algorythmus durchführen
                if (averageVorne <=
                    0.2) {             //Turtlebot hat eine Wand vor sich und eine Wand rechts neben sich
                    if (averageLinks <=
                        0.25) {             //Kommentar aus Zeile 153 + noch eine Wand dicht auf der Linken Seite
                        ROS_INFO("Erste Switch");
                        switch (Richtungsgeber) {
                            case 0:
                                //ROS_INFO("HALLO: %f", WirdUmgenannt);
                                test37 = 2;
                                if (WirdUmgenannt < 180.0) {
                                    robot_move(NEUNZIG_LINKS);

                                }

                                if (WirdUmgenannt > 180.0) {
                                    Richtungsgeber = 180;

                                }
                                ros::spinOnce();
                                break;
                            case 90:
                                test37 = 2;
                                while (WirdUmgenannt <= 180.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();

                                }
                                while (WirdUmgenannt < -90.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();

                                }
                                Richtungsgeber = -90.0;
                                break;
                            case 180:
                                test37 = 2;
                                if (WirdUmgenannt > 0.0) {
                                    while (WirdUmgenannt > 0.0) {
                                        robot_move(NEUNZIG_LINKS);
                                        ros::spinOnce();

                                    }
                                }
                                while (WirdUmgenannt < 0.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();

                                }
                                Richtungsgeber = 0.0;
                                break;
                            case -90:
                                test37 = 2;
                                while (WirdUmgenannt <= 0.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();

                                }
                                while (WirdUmgenannt < 90.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();

                                }
                                Richtungsgeber = 90.0;
                                break;
                        }                                                 //180° Linksdrehung da der Turtlebot in einer Sackgasse ist
                    } else {                              //Links hat der Turtlebot keine Wand
                        ROS_INFO("zweite Switch");
                        switch (Richtungsgeber) {
                            case 0:
                                test37 = 1;
                                ROS_INFO("DREHUNGSWINKEL: %f", WirdUmgenannt);
                                while (WirdUmgenannt < 90.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();
                                }
                                ros::spinOnce();
                                break;
                            case 90:
                                ROS_INFO("DREHUNGSWINKEL90: %f", WirdUmgenannt);
                                test37 = 1;
                                while (WirdUmgenannt < 180 && WirdUmgenannt > 0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();
                                }
                                Richtungsgeber = 180;
                                break;
                            case 180:
                                test37 = 1;
                                if (WirdUmgenannt > 0.0) {
                                    while (WirdUmgenannt > 0.0) {
                                        robot_move(NEUNZIG_LINKS);
                                        ros::spinOnce();
                                    }
                                }
                                while (WirdUmgenannt < -90.0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();
                                }
                                Richtungsgeber = -90;
                                ros::spinOnce();
                                break;
                            case -90:
                                test37 = 1;
                                while (WirdUmgenannt < 0) {
                                    robot_move(NEUNZIG_LINKS);
                                    ros::spinOnce();
                                }
                                Richtungsgeber = 0;
                                ros::spinOnce();
                                break;
                        }                                 //Linksdrehung um 90°
                    }
                } else {                              //Turtlebot hat keine Wand vor sich aber rechts neben sich, daher kann er geradeaus fahren
                    //while(averageVorne > 0.2 && (averageRechts < 0.25 || averageLinks < 0.25) ){
                    robot_move(GERADEAUS_LANG);
                    ros::spinOnce();
                    //}
                }
            } else {
                ROS_INFO("KJHGBSDKJGADKHJAGKJHDGAKJHSGKJHASGFKHJSAGFKHSAGKFHGSKAJFGSKAHJGFKJHSAKJHDGSAJ")  ;
                ROS_INFO("faelle %i:", faelle);//Turtlebot hat keine Wand rechts neben sich daher ein Gang oder eine Tür
                switch (faelle) {

                    case 0:
                        test37 = 3;
                        faelle = 4;
                        ros::spinOnce();
                        break;
                        /*case 1:

                            test37 = 3;
                            ros::spinOnce();
                            break;

                        case 2:
                            robot_move(GERADEAUS_MITTEL);
                            faelle = 0;
                            break;*/
                }                                  //Turtlebot dreht sich 90° nach rechts und bewegt sich ein Stück nach vorne
            }
        }


    }
//}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Camera position in map frame

    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;


    if (koordinatenVorwaerts == 0) {
        rechneryy = (msg->pose.pose.position.y);


        rechnerxx = (msg->pose.pose.position.x);


    }
    if (faelle == 4 || faelle == 2) { //setzen des 1. Y und X Wertes
        koordinatenVorwaerts = 1;
        previousX = (msg->pose.pose.position.x); //setzen der aktuellen X und Y Positionen


        previousy = (msg->pose.pose.position.y);


        weg = sqrt(pow((previousX - rechnerxx), 2) +
                   pow((previousy - rechneryy), 2)); //Formel zum berechnen der Entfernung zwischen 2 Punkten

    }


    if (!(faelle == 4) && !(faelle == 2)) {
        weg = 0;
    }

   // ROS_INFO("WEG 2. CHECKPOINT: %f", weg);
    //ROS_INFO("CHECKPOINT 2 RECHNERXX: %f", rechnerxx);
    //ROS_INFO("CHECKPOINT 2 RECHNERYY: %f", rechneryy);
    //ROS_INFO("CHECKPOINT 2 PREVIOUS X: %f", previousX);
    //ROS_INFO("CHECKPOINT 2 PREVIOUS Y: %f", previousy);
    // Orientation quaternion
    tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Output the measure
    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             tx, ty, tz,
             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

    WirdUmgenannt = yaw * RAD2DEG;

}


int main(int argc, char **argv) {
    // Initialize a ROS node

    ros::init(argc, argv, "node");
    ros::NodeHandle n;
    ros::Rate rateH(5);
    ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);
    ros::Subscriber odom_subscriber = n.subscribe("/odom", 10, odomCallback);

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    float test1 = averageVorne;

    while (ros::ok()) {
        geometry_msgs::Twist msg;

        ros::spin();
    }

}
