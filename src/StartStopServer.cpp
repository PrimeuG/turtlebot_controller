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
#include <stack>

#define RAD2DEG 57.295779513
using namespace std_msgs;

//https://github.com/cohnsted1/TurtleBot--MazeSolver/blob/main/src/new_main.cpp

ros::Publisher motor_command_publisher;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;


static int counter = 1;
static float aktuelleRichtung = 0; //aktueller Winkel
static int Richtungsgeber = 180;
static float averageVorne = 0.0;
static float averageLinks = 0.0;
static float averageRechts = 0.0;
static float averageVorneRechts = 0.0;
static int rechtsAuswahl = 0;
static int bewegungstyp = 0;  //Bewegungstyp 0 Laser, Typ 1 90 Links, Typ 2 180 Grad, Typ 3 Rechtskurve
static int koordinatenVorwaerts = 0;


float x_fest = 0; //erster X Wert
float y_fest = 0; //erster Y Wert
float x_aktuell = 0; //aktuellste X Position
float y_aktuell = 0;//aktuellste X Position
float weg = 0;


// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    GERADEAUS,
    NEUNZIG_LINKS,
    NEUNZIG_RECHTS


} ROBOT_MOVEMENT;


bool robot_move(const ROBOT_MOVEMENT move_type) {

    if (move_type == STOP) {

        ROS_INFO("STOP! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
        motor_command_publisher.publish(motor_command);

    } else if (move_type == GERADEAUS) {

        ROS_INFO("Geradeaus! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.05;
        motor_command_publisher.publish(motor_command);

    } else if (move_type == NEUNZIG_LINKS) {

        ROS_INFO("Neunzig Links! \n");

        motor_command.linear.x = 0.0;
        motor_command.angular.z = 0.1;
        motor_command_publisher.publish(motor_command);

    } else if (move_type == NEUNZIG_RECHTS) {

        ROS_INFO("Neunzig Rechts! \n");

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
    ROS_INFO("Lasercallback rechtsAuswahl: %i", rechtsAuswahl);

    ros::Rate rateH(3);
    laser_msg = *msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;


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
        //ROS_INFO("Counter: %i", counter);


        if (bewegungstyp == 1) {

            switch (Richtungsgeber) {

                case 0:

                    //ROS_INFO("HALLO: %f", aktuelleRichtung);

                    while (aktuelleRichtung < 90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 90;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case 90:

                    while (aktuelleRichtung < 180 && aktuelleRichtung > 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 180;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case 180:

                    while (aktuelleRichtung > 0.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    while (aktuelleRichtung < -90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = -90;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case -90:

                    while (aktuelleRichtung < 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 0;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

            }

        }


        if (bewegungstyp == 2) {

            switch (Richtungsgeber) {

                case 0:

                    while (aktuelleRichtung < 180.0 && aktuelleRichtung > -45.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    if (aktuelleRichtung < -5) {
                        ROS_INFO("aktuelle Richtung ist richtig mies! %f", aktuelleRichtung);
                    }

                    Richtungsgeber = 180;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case 90:

                    while (aktuelleRichtung <= 180.0 && aktuelleRichtung > 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    while (aktuelleRichtung < -90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = -90.0;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case 180:


                    while (aktuelleRichtung > 0.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    while (aktuelleRichtung < 0.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 0;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case -90:

                    while (aktuelleRichtung <= 0.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    while (aktuelleRichtung < 90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 90;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

            }
        }

        if (bewegungstyp == 3) {

            ROS_INFO("rechtsAuswahl bewegungstyp 3: %i", rechtsAuswahl);
            ROS_INFO("aktuelleRichtung: %f", aktuelleRichtung);
            ROS_INFO("Richtungsgeber: %i", Richtungsgeber);


            if (rechtsAuswahl == 4) {

                if (averageVorne < 0.18) {
                    ROS_INFO("NOTSTOPPER 1!!!!!!!!!!!!!!!!!!!!!!");
                    rechtsAuswahl = 1;
                    koordinatenVorwaerts = 0;
                    ros::spinOnce();
                }

                while (weg < 0.13 && weg > -0.13) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }

                rechtsAuswahl = 1;
                koordinatenVorwaerts = 0;
                ros::spinOnce();

            } else if (rechtsAuswahl == 1) {

                switch (Richtungsgeber) {

                    case 0:

                        while (aktuelleRichtung > -90.0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = -90;
                        ros::spinOnce();
                        break;

                    case 90:

                        while (aktuelleRichtung > 0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = 0;
                        ros::spinOnce();
                        break;

                    case 180:

                        while (aktuelleRichtung > 90 || aktuelleRichtung < 0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = 90;
                        ros::spinOnce();
                        break;

                    case -90:

                        while (aktuelleRichtung > -179 && aktuelleRichtung < 1) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = 180;
                        ros::spinOnce();
                        break;

                }

            } else if (rechtsAuswahl == 2) {

                if (averageVorne < 0.18) {
                    ROS_INFO("NOTSTOPPER 2!!!!!!!!!!!!!!!!!!!!!!");
                    rechtsAuswahl = 0;
                    koordinatenVorwaerts = 0;
                    bewegungstyp = 0;
                    ros::spinOnce();
                }

                while (weg < 0.13 && weg > -0.13) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }

                koordinatenVorwaerts = 0;
                rechtsAuswahl = 0;
                bewegungstyp = 0;
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
                            while (aktuelleRichtung < 90.0) {
                                robot_move(NEUNZIG_LINKS);
                                bewegungstyp = 1;
                                ros::spinOnce();
                            }
                            Richtungsgeber = 90;
                            break;
                        case 90:
                            while (aktuelleRichtung < 180 && aktuelleRichtung > 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 180;
                            break;
                        case 180:
                            if (aktuelleRichtung > 0.0) {
                                while (aktuelleRichtung > 0.0) {
                                    robot_move(NEUNZIG_LINKS);
                                }
                            }
                            while (aktuelleRichtung < -90.0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = -90;
                            break;
                        case -90:
                            while (aktuelleRichtung < 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 0;
                            break;
                    }                               //180° Linksdrehung, da Wand links vorhanden ist
                    counter = 1;                    //um in if-Bedingung (counter ==1) zu kommen
                } else {
                    switch (Richtungsgeber) {
                        case 0:
                            while (aktuelleRichtung < 90.0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 90;
                            break;
                        case 90:
                            while (aktuelleRichtung < 180 && aktuelleRichtung > 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 180;
                            break;
                        case 180:
                            if (aktuelleRichtung > 0.0) {
                                while (aktuelleRichtung > 0.0) {
                                    robot_move(NEUNZIG_LINKS);
                                }
                            }
                            while (aktuelleRichtung < -90.0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = -90;
                            break;
                        case -90:
                            while (aktuelleRichtung < 0) {
                                robot_move(NEUNZIG_LINKS);
                            }
                            Richtungsgeber = 0;
                            break;
                    }                       //90° Linksdrehung
                    counter = 1;
                }
            } else {
                robot_move(
                        GERADEAUS);                              //Turtlebot fährt dicht zur ersten Wand ab der er sich orientieren kann
            }
        }*/ //if {                             //ab hier an kann der Turtlebot sich orientieren
        if (averageRechts <=
            0.275) {     //Turtlebot hat rechts neben sich eine Wand und kann somit den Rechte-Hand Algorythmus durchführen

            if (averageVorne <= 0.2) {    //Turtlebot hat eine Wand vor sich und eine Wand rechts neben sich

                if (averageLinks <= 0.25) {   //Kommentar aus Zeile davor + noch eine Wand dicht auf der Linken Seite

                    ROS_INFO("Erster Switch");

                    switch (Richtungsgeber) {

                        case 0:

                            bewegungstyp = 2;
                            ros::spinOnce();
                            break;

                        case 90:

                            bewegungstyp = 2;
                            ros::spinOnce();
                            break;

                        case 180:

                            bewegungstyp = 2;
                            ros::spinOnce();
                            break;

                        case -90:

                            bewegungstyp = 2;
                            ros::spinOnce();
                            break;

                    }  //180° Linksdrehung da der Turtlebot in einer Sackgasse ist

                } else {   //Links hat der Turtlebot keine Wand

                    ROS_INFO("Zweiter Switch");

                    switch (Richtungsgeber) {

                        case 0:

                            bewegungstyp = 1;
                            ros::spinOnce();
                            break;

                        case 90:

                            bewegungstyp = 1;
                            ros::spinOnce();
                            break;

                        case 180:

                            bewegungstyp = 1;
                            ros::spinOnce();
                            break;

                        case -90:

                            bewegungstyp = 1;
                            ros::spinOnce();
                            break;

                    }                                 //Linksdrehung um 90°
                }

            } else {                              //Turtlebot hat keine Wand vor sich aber rechts neben sich, daher kann er geradeaus fahren

                robot_move(GERADEAUS);
                ros::spinOnce();

            }

        } else {
            //Turtlebot hat keine Wand rechts neben sich daher ein Gang oder eine Tür

            if (rechtsAuswahl != 0) {
                ROS_INFO("Rechtsauswahl ist nicht null ! sondern: %i", rechtsAuswahl);
            }

            if (averageVorne <= 0.2) {  //Keine Wand rechts neben sich, aber vor sich
                ROS_INFO("ZULU  !!!!");

                rechtsAuswahl = 1;
                bewegungstyp = 3;
                ros::spinOnce();

            } else if (averageVorne >
                       0.2) {    //Turtlebot faehrt vorwaerts dreht sich 90° nach rechts und bewegt sich ein Stück nach vorne

                if (rechtsAuswahl != 0) {
                    ROS_INFO("Rechtsauswahl ist nicht null ! sondern: %i", rechtsAuswahl);
                }

                bewegungstyp = 3;
                rechtsAuswahl = 4;
                ros::spinOnce();

            }
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;

    if (koordinatenVorwaerts == 0) {
        y_fest = (msg->pose.pose.position.y);

        x_fest = (msg->pose.pose.position.x);

    }

    if (rechtsAuswahl == 4 || rechtsAuswahl == 2) { //setzen des 1. Y und X Wertes
        koordinatenVorwaerts = 1;
        x_aktuell = (msg->pose.pose.position.x); //setzen der aktuellen X und Y Positionen

        y_aktuell = (msg->pose.pose.position.y);

        weg = sqrt(pow((x_aktuell - x_fest), 2) +
                   pow((y_aktuell - y_fest), 2)); //Formel zum berechnen der Entfernung zwischen 2 Punkten

    }

    if (rechtsAuswahl != 4 && rechtsAuswahl != 2) {
        weg = 0;
    }

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

    aktuelleRichtung = yaw * RAD2DEG;

}


int main(int argc, char **argv) {
    // Initialize a ROS node

    ros::init(argc, argv, "node");
    ros::NodeHandle n;

    ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);
    ros::Subscriber odom_subscriber = n.subscribe("/odom", 10, odomCallback);

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    while (ros::ok()) {
        geometry_msgs::Twist msg;
        ros::spin();
    }

}
