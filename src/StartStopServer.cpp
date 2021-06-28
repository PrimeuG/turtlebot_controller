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


static int counter = 0;
static float aktuelleRichtung = 0; //aktueller Winkel
static int Richtungsgeber = 0;
static float averageVorne = 0.0;
static float averageLinks = 0.0;
static float averageRechts = 0.0;
static float averageVorneRechts = 0.0;
static int rechtsAuswahl = 0;
static int bewegungstyp = 0;  //Bewegungstyp 0 Laser, Typ 1 90 Links, Typ 2 180 Grad, Typ 3 Rechtskurve, Typ 4 Drehung fuer Anfahrt
static int koordinatenVorwaerts = 0;
static std::string MoveAusgabe;
static int countercounter = 0;
static int halter = 0;
static int aktualisierer = 2;


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
    MoveAusgabe = move_type;
    if (move_type == STOP) {

        ROS_INFO("STOP! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
        motor_command_publisher.publish(motor_command);


    } else if (move_type == GERADEAUS) {

        if (averageVorne <= 0.14) {
            motor_command.angular.z = 0.0;
            motor_command.linear.x = 0.0;
            motor_command_publisher.publish(motor_command);
            ros::spinOnce();
        }

        //ROS_INFO("Geradeaus! \n");
        switch (Richtungsgeber) {

            case 0:
                //ROS_INFO("AKTUELLE RICHTUNG CASE 0 %f", aktuelleRichtung);
                while (aktuelleRichtung > -0.5 && aktuelleRichtung < 0.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.05;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung <= -0.5) {
                    motor_command.angular.z = 0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

                while (aktuelleRichtung >= 0.5) {
                    motor_command.angular.z = -0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

            case 90:


                //ROS_INFO("AKTUELLE RICHTUNG CASE 90 %f", aktuelleRichtung);
                while (aktuelleRichtung < 90.5 && aktuelleRichtung > 89.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.05;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung >= 90.5) {
                    motor_command.angular.z = -0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

                while (aktuelleRichtung <= 89.5) {
                    motor_command.angular.z = 0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

            case 180:
                // ROS_INFO("AKTUELLE RICHTUNG CASE 180 %f", aktuelleRichtung);
                while (aktuelleRichtung > 179.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.05;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung > -179.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.05;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung <= 179.5 && aktuelleRichtung > 100) {
                    motor_command.angular.z = 0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

                while (aktuelleRichtung <= -179.5) {
                    motor_command.angular.z = -0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

            case -90:
                // ROS_INFO("AKTUELLE RICHTUNG CASE -90 %f", aktuelleRichtung);
                while (aktuelleRichtung > -90.5 && aktuelleRichtung < -89.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.05;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung <= -90.5) {
                    motor_command.angular.z = 0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

                while (aktuelleRichtung >= -89.5) {
                    motor_command.angular.z = -0.02;
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }


        }


    } else if (move_type == NEUNZIG_LINKS) {

        // ROS_INFO("Neunzig Links! \n");

        motor_command.linear.x = 0.0;
        motor_command.angular.z = 0.1;
        motor_command_publisher.publish(motor_command);

    } else if (move_type == NEUNZIG_RECHTS) {

        //ROS_INFO("Neunzig Rechts! \n");

        motor_command.linear.x = 0.0;
        motor_command.angular.z = -0.1;
        motor_command_publisher.publish(motor_command);

    } else {
        ROS_INFO("Move type wrong! \n");
        return false;
    }

    //usleep(10);
    return true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {

    /*ROS_INFO("Richtungsgeber %i", Richtungsgeber);
    ROS_INFO("Lasercallback rechtsAuswahl: %i", rechtsAuswahl);
    ROS_INFO("RobotMove Befehl: %S", MoveAusgabe.c_str());*/

    ros::Rate rateH(7);
    //rateH.sleep();
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

    for (int z = 270; z < 276; z++) {
        summeRechts += laser_ranges[z];

        if (z == 275) {
            averageRechts = summeRechts / 6.0;
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
        averageRechts > 3.5) {
        ROS_INFO("STÖRUNG");
        ROS_INFO("averageRechts: %f", averageRechts);
        ROS_INFO("averageLinks: %f", averageLinks);
        ROS_INFO("averageVorne: %f", averageVorne);
        ROS_INFO("averageVorneRechts: %f", averageVorneRechts);
        ros::spinOnce();
    } else {
        /*ROS_INFO("averageRechts: %f", averageRechts);
        ROS_INFO("averageLinks: %f", averageLinks);
        ROS_INFO("averageVorne: %f", averageVorne);*/
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
                        // ROS_INFO("aktuelle Richtung ist richtig mies! %f", aktuelleRichtung);
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

            /*ROS_INFO("rechtsAuswahl bewegungstyp 3: %i", rechtsAuswahl);
            ROS_INFO("aktuelleRichtung: %f", aktuelleRichtung);
            ROS_INFO("Richtungsgeber: %i", Richtungsgeber);*/


            if (rechtsAuswahl == 4) {

                if (averageVorne < 0.18) {
                    ROS_INFO("NOTSTOPPER 1!!!!!!!!!!!!!!!!!!!!!!");
                    rechtsAuswahl = 1;
                    koordinatenVorwaerts = 0;
                    ros::spinOnce();
                }

                while (weg < 0.12 && weg > -0.12) {
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
                    halter = 5;
                    aktualisierer = 2;
                    ros::spinOnce();
                }

                while (weg < 0.16 && weg > -0.16) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }
                aktualisierer = 2;
                koordinatenVorwaerts = 0;
                rechtsAuswahl = 0;
                bewegungstyp = 0;
                halter = 5;
                //rateH.sleep();
                ros::spinOnce();

            }
        }

        if (bewegungstyp == 4) {
            switch (Richtungsgeber) {

                case 0:

                    //ROS_INFO("HALLO: %f", aktuelleRichtung);

                    while (aktuelleRichtung < 90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }
                    Richtungsgeber = 90;
                    bewegungstyp = 0;
                    countercounter = 2;
                    ros::spinOnce();
                    break;

                case 90:

                    while (aktuelleRichtung < 180 && aktuelleRichtung > 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 180;
                    bewegungstyp = 0;
                    countercounter = 2;
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
                    countercounter = 2;
                    ros::spinOnce();
                    break;

                case -90:

                    while (aktuelleRichtung < 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 0;
                    bewegungstyp = 0;
                    countercounter = 2;
                    ros::spinOnce();
                    break;

            }

        }
        //ros::spinOnce();
    }
    //ROS_INFO("Halter: %i", halter);
    if (halter > 0) {
        ROS_INFO("Halter: %i", halter);
        halter--;
        rateH.sleep();
        ros::spinOnce();
    } else if (halter == 0) {


        if (counter == 0) {//Ist für den Anfang damit der Turtlebot von der mitte aus zur ersten Wand fährt

            if (countercounter == 0) {
                while (averageVorne > 0.19) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }
                if (averageVorne <= 0.19) {
                    countercounter = 1;
                }
            }

            if (countercounter == 1) {
                bewegungstyp = 4;
                ros::spinOnce();
            }

            if (countercounter == 2) {
                counter = 1;
                ros::spinOnce();
            }


        } else if (counter == 1) {                             //ab hier an kann der Turtlebot sich orientieren
            if (bewegungstyp == 0) {
                ROS_INFO("ZEILE 636");
                ROS_INFO("averageRechts: %f", averageRechts);
                ROS_INFO("averageLinks: %f", averageLinks);
                ROS_INFO("averageVorne: %f", averageVorne);
                if (averageRechts <=
                    0.275) {     //Turtlebot hat rechts neben sich eine Wand und kann somit den Rechte-Hand Algorythmus durchführen
                    ROS_INFO("RECHTS KLEINER ALS 0.275");
                    if (averageVorne <= 0.2) {    //Turtlebot hat eine Wand vor sich und eine Wand rechts neben sich
                        ROS_INFO("VORNE KLEINER ALS 0.2");
                        if (averageLinks <=
                            0.25) {   //Kommentar aus Zeile davor + noch eine Wand dicht auf der Linken Seite
                            ROS_INFO("LINKS KLEINER ALS 0.25");
                            //ROS_INFO("Erster Switch");

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
                            ROS_INFO("LINKS GROESSER ALS 0.25");
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
                        ROS_INFO("VORNE GROESSER ALS 0.2");
                        robot_move(GERADEAUS);
                        ros::spinOnce();

                    }

                } else {
                    if (aktualisierer > 0) {
                        aktualisierer--;
                        ROS_INFO("aktualisierer MINUS");
                        ros::spinOnce();
                    } else if (aktualisierer == 0) {

                        ROS_INFO("RECHTS GROESSER ALS 0.275");
                        //Turtlebot hat keine Wand rechts neben sich daher ein Gang oder eine Tür

                        if (rechtsAuswahl != 0) {
                            ROS_INFO("Rechtsauswahl ist nicht null ! sondern: %i", rechtsAuswahl);
                        }

                        if (averageVorne <= 0.2) {  //Keine Wand rechts neben sich, aber vor sich
                            ROS_INFO("RECHTSDREHUNG DANN GERADE");
                            if (rechtsAuswahl == 0) {
                                rechtsAuswahl = 1;
                                bewegungstyp = 3;
                                ros::spinOnce();
                            }

                        } else if (averageVorne >
                                   0.2) {    //Turtlebot faehrt vorwaerts dreht sich 90° nach rechts und bewegt sich ein Stück nach vorne

                            if (rechtsAuswahl != 0) {
                                ROS_INFO("Rechtsauswahl ist nicht null ! sondern: %i", rechtsAuswahl);
                            }
                            ROS_INFO("GERADE, RECHTSDREHUNG DANN GERADE");
                            if (rechtsAuswahl == 0) {
                                rechtsAuswahl = 4;
                                bewegungstyp = 3;
                                ros::spinOnce();
                            }

                        }
                    }
                }
            } //else ros::spinOnce();
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


    ros::spinOnce();
}


int main(int argc, char **argv) {
    // Initialize a ROS node

    ros::init(argc, argv, "node");
    ros::NodeHandle n;

    ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 5, laserCallback);
    ros::Subscriber odom_subscriber = n.subscribe("/odom", 5, odomCallback);

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    while (ros::ok()) {
        geometry_msgs::Twist msg;
        ros::spinOnce();
    }

    ros::spinOnce();

}
