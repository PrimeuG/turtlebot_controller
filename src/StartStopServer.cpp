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

ros::Publisher motor_command_publisher;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;
//Zeile 34 und 59 auskommentiert!!!

static int counter = 0; //Countervariable zum festlegen das Roboter vom Start aus erste Wand erreicht hat, wird auf 1 gesetzt sobald dies geschehen ist
static float aktuelleRichtung = 0; //aktueller Winkel
static int Richtungsgeber = 0;  //Variable zum setzen der letzten Ausrichtungsposition, kann 0, 90, 180 oder -90 sein
static float averageVorne = 0.0; //Durchschnitt der Laserwerte in einem gewissen Radius vor dem Roboter
static float averageLinks = 0.0; //Durchschnitt der Laserwerte in einem gewissen Radius links neben dem Roboter
static float averageRechts = 0.0; //Durchschnitt der Laserwerte in einem gewissen Radius rechts neben dem Roboter
static float averageVorneRechts = 0.0; //Durchschnitt der Laserwerte in einem gewissen Radius vorne Rechts neben dem Roboter
static int rechtsAuswahl = 0; //Variable zum feststellen ob geradeaus rechts geradeaus oder rechts geradeaus gefahren werden soll, wird auf 4 gesetzt zu beginn wenn erster Fall auf 1 wenn zweiter Fall
static int bewegungstyp = 0;  //Bewegungstyp 0 Laser, Typ 1 90 Links, Typ 2 180 Grad, Typ 3 Rechtskurve, Typ 4 Drehung fuer Anfahrt
static int koordinatenVorwaerts = 0; //legt Startpunkt zur Wegberechnung fest wenn Wert von 0 auf 1 springt
//static std::string MoveAusgabe;
static int countercounter = 0; //Variable zum Bewegungsablauf koordinieren, 0 heißt geraudeaus, 1 linksdrehung und 2 ist abschluss
static int halter = 0; //Variable zum verzögern der Messwerte
static int aktualisierer = 2; //Variable zum verzögern der Messwerte


float x_fest = 0; //erster X Wert
float y_fest = 0; //erster Y Wert
float x_aktuell = 0; //aktuellste X Position
float y_aktuell = 0;//aktuellste X Position
float weg = 0; //Weg von Koordinaten berechnet zur Kontrolle wie weit der Turtlebot gefahren ist


// Richtungen die der Roboter waehlen kann
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    GERADEAUS,
    NEUNZIG_LINKS,
    NEUNZIG_RECHTS


} ROBOT_MOVEMENT;

//Bewegungsmethode des Roboters
bool robot_move(const ROBOT_MOVEMENT move_type) {
    //MoveAusgabe = move_type;
    if (move_type == STOP) { //Stopp Bewegungsdefinition des Roboters

        ROS_INFO("STOP! \n");

        motor_command.angular.z = 0.0; //Geschwindigkeit wird auf 0 gesetzt und Befehl wird gepublished
        motor_command.linear.x = 0.0;
        motor_command_publisher.publish(motor_command);


    } else if (move_type == GERADEAUS) {    //Falls Befehl GERADEAUS lautet benutzt er diese If Bedingung

        if (averageVorne <= 0.14) {         //Notbremse greift wenn Abstand zur Wand vor dem Turtlebot kleiner als 0.14 ist, gibt dann Befehl zum anhalten
            motor_command.angular.z = 0.0;
            motor_command.linear.x = 0.0;
            motor_command_publisher.publish(motor_command);
            ros::spinOnce();
        }

        //ROS_INFO("Geradeaus! \n");
        switch (Richtungsgeber) {           //Bezieht den Richtungsgeber ein um festzustellen in welche Richtung der Turtlebot orientiert ist
                                            //nutzt dann je nach Richtungsgeberwert andere Werte um dafuer zu sorgen, dass der Roboter sich nicht vom Kurs entfernt
            case 0:
                //ROS_INFO("AKTUELLE RICHTUNG CASE 0 %f", aktuelleRichtung);
                while (aktuelleRichtung > -0.5 && aktuelleRichtung < 0.5) { //solange die aktuelle Richtung innerhalb von 0,5 Grad zur Zielrichtung liegt faehrt der Roboter geraudeaus
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.045;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung <= -0.5) {    //Liegt die tatsaechliche Richtung von der Zielrichtung weiter entfernt als -0.5 Grad nach Rechts, dreht sich der Roboter
                    motor_command.angular.z = 0.02;    //waehrend er faehrt um 0,02  nach Links waehrenddessen
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

                while (aktuelleRichtung >= 0.5) {    //Liegt die tatsaechliche Richtung von der Zielrichtung weiter entfernt als 0.5 Grad nach Links, dreht sich der Roboter
                    motor_command.angular.z = -0.02; //waehrend er faehrt um -0,02  nach Rechts waehrenddessen
                    motor_command.linear.x = 0.02;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();

                }

            case 90:  //Aufgabe Analog zu case 0


                //ROS_INFO("AKTUELLE RICHTUNG CASE 90 %f", aktuelleRichtung);
                while (aktuelleRichtung < 90.5 && aktuelleRichtung > 89.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.045;
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

            case 180:  //Aufgabe Analog zu case 0
                // ROS_INFO("AKTUELLE RICHTUNG CASE 180 %f", aktuelleRichtung);
                while (aktuelleRichtung > 179.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.045;
                    motor_command_publisher.publish(motor_command);
                    ros::spinOnce();
                }

                while (aktuelleRichtung > -179.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.045;
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

            case -90:  //Aufgabe Analog zu case 0
                // ROS_INFO("AKTUELLE RICHTUNG CASE -90 %f", aktuelleRichtung);
                while (aktuelleRichtung > -90.5 && aktuelleRichtung < -89.5) {
                    motor_command.angular.z = 0.0;
                    motor_command.linear.x = 0.045;
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


    } else if (move_type == NEUNZIG_LINKS) { //Falls Move Befehl NEUNZIG_LINKS ist dreht er sich mit 0,1 Rad

        // ROS_INFO("Neunzig Links! \n");

        motor_command.linear.x = 0.0;
        motor_command.angular.z = 0.1;
        motor_command_publisher.publish(motor_command);

    } else if (move_type == NEUNZIG_RECHTS) {  //Falls Move Befehl NEUNZIG_LINKS ist dreht er sich mit 0,1 Rad

        //ROS_INFO("Neunzig Rechts! \n");

        motor_command.linear.x = 0.0;
        motor_command.angular.z = -0.1;
        motor_command_publisher.publish(motor_command);

    } else { //Fehler abfangen falls Move Command falsch gesendet wird
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

    //Float Werte von den eingelesen Laserwerten, welche aufaddiert werden fuer die jeweilige Richtung
    float summeVorne = 0.0;
    float summeLinks = 0.0;
    float summeRechts = 0.0;
    float summeVorneRechts = 0.0;

    //fasst Werte von 0 bis 5 Grad ins Array
    for (int z = 0; z < 6; z++) {
        summeVorne += laser_ranges[z];
    }
    //fasst Werte von 355 bis 359 Grad ins Array
    for (int k = 355; k < 360; k++) {
        summeVorne += laser_ranges[k];
        //Berechnet den Durchschnitt der eingelesenen Laserwerte
        if (k == 359) {
            averageVorne = summeVorne / 11.0;
        }
    }
    //Analog zur Berechnung von summeVorne bzw. averageVorne
    for (int z = 85; z < 96; z++) {
        summeLinks += laser_ranges[z];

        if (z == 95) {
            averageLinks = summeLinks / 11.0;
        }
    }
    //Analog zur Berechnung von summeVorne bzw. averageVorne
    for (int z = 270; z < 276; z++) {
        summeRechts += laser_ranges[z];

        if (z == 275) {
            averageRechts = summeRechts / 6.0;
        }
    }

    //Analog zur Berechnung von summeVorne bzw. averageVorne
    for (int z = 340; z < 346; z++) {
        summeVorneRechts += laser_ranges[z];

        if (z == 345) {
            averageVorneRechts = summeVorneRechts / 6.0;
        }
    }


    //ROS_INFO("VORNE %f", averageVorne);
    if (false) {        //Kontrollbedingung fuer die Werte, jedoch unpraktikabel da wenn Zielblock entfernt wird Infitinity ausgegeben wird, deswegen aktuell auf false gesetzt
        //averageVorne <= 0 || averageVorne > 3.5 || averageLinks <= 0 || averageLinks > 3.5 || averageRechts <= 0 ||
        //        averageRechts > 3.5
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

        //Wenn Beweguungstyp auf 1 gesetzt wurde geht er hier rein
        if (bewegungstyp == 1) {
            //Richtungsgeber ist wieder ausschlaggebend
            switch (Richtungsgeber) {

                case 0:

                    //ROS_INFO("HALLO: %f", aktuelleRichtung);

                    while (aktuelleRichtung < 90.0) { //Solange die gewaehlte Richtung, in diesem Fall 90 Grad nicht erreicht ist, dreht sich der Roboter nach Links
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 90;           //Wenn gewuenschte Richtung erreicht ist wird der Richtungsgeber auf den entsprechenden Wert gesetzt
                    bewegungstyp = 0;               //Bewegungstyp wurde auf 0 gesetzt
                    ros::spinOnce();
                    break;

                case 90:
                    //Analog zu case 0
                    while (aktuelleRichtung < 180 && aktuelleRichtung > 0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    Richtungsgeber = 180;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case 180:
                    //Analog zu case 0
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
                    //Analog zu case 0
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

        //Wenn Bewgungstyp 2 ist wird diese Bedingung erfuellt
        if (bewegungstyp == 2) {
            //Richtungsgeber ist entscheidend, Switch wird nach passendem case druchsucht
            switch (Richtungsgeber) {

                case 0:
                    //Solange aktuelle Richtung kleiner als 180 Grad ist, und groesser als -45 Grad dreht er sich links
                    while (aktuelleRichtung < 180.0 && aktuelleRichtung > -45.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }

                    if (aktuelleRichtung < -5) {
                        // ROS_INFO("aktuelle Richtung ist richtig mies! %f", aktuelleRichtung);
                    }
                    //der Richtungsgeber wird entsprechend der 180 Grad Drehung (von 0 Grad aus) auf 180 gesetzt, sowie der Bewegungstyp auf 0
                    Richtungsgeber = 180;
                    bewegungstyp = 0;
                    ros::spinOnce();
                    break;

                case 90:
                    //Analog zu case 0
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
                    //Analog zu case 0
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
                    //Analog zu case 0
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
        //Wenn Bewgungstyp 3 ist wird diese Bedingung erfuellt
        if (bewegungstyp == 3) {

            /*ROS_INFO("rechtsAuswahl bewegungstyp 3: %i", rechtsAuswahl);
            ROS_INFO("aktuelleRichtung: %f", aktuelleRichtung);
            ROS_INFO("Richtungsgeber: %i", Richtungsgeber);*/

            //Falls rechtsAuswahl4 ist startet der Ablauf hier
            if (rechtsAuswahl == 4) {
                //Falls der Abstand zur Wand vor dem Roboter kleiner als 0,18 wird macht er eine Notbremsung und geht zum nächsten Schritt über des Bewegungsablaufes indem rechtsAuswahl auf 1 gesetzt wird
                if (averageVorne < 0.18) {
                    ROS_INFO("NOTSTOPPER 1!!!!!!!!!!!!!!!!!!!!!!");
                    rechtsAuswahl = 1;
                    koordinatenVorwaerts = 0;   //Weg zur Entfernungsberechnung wird zurückgesetzt
                    ros::spinOnce();
                }
                //solange die gefahrene Distanz nicht 0,12/-0,12 erreicht faehrt der Roboter geradeaus
                while (weg < 0.12 && weg > -0.12) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }
                //Sobald Roboter gefahrene Distanz zurueckgelegt hat wird rechtsAuswahl auf 1 gesetzt und die Koordinaten zur Wegberechnung zurueckgesetzt
                rechtsAuswahl = 1;
                koordinatenVorwaerts = 0;
                ros::spinOnce();
            //Falls Rechtsauswahl 1 ist wird diese Bedingung wahr
            } else if (rechtsAuswahl == 1) {
                //Richtungsgeber Switch zur aktuellen Position wegen Rechtsdrehung
                switch (Richtungsgeber) {

                    case 0:
                        //Solange der Roboter sich nicht um 90 Grad nach rechts gedreht hat, dreht er sich nach rechts
                        while (aktuelleRichtung > -90.0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }
                        //sobald Roboter sich 90 Grad gedreht hat wird rechtsAuswahl auf 2 gesetzt und der Richtungsgeber auf neu gedrehte Richtung
                        rechtsAuswahl = 2;
                        Richtungsgeber = -90;
                        ros::spinOnce();
                        break;

                    case 90:
                        //Analog zu case 0
                        while (aktuelleRichtung > 0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = 0;
                        ros::spinOnce();
                        break;

                    case 180:
                        //Analog zu case 0
                        while (aktuelleRichtung > 90 || aktuelleRichtung < 0) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = 90;
                        ros::spinOnce();
                        break;

                    case -90:
                        //Analog zu case 0
                        while (aktuelleRichtung > -179 && aktuelleRichtung < 1) {
                            robot_move(NEUNZIG_RECHTS);
                            ros::spinOnce();
                        }

                        rechtsAuswahl = 2;
                        Richtungsgeber = 180;
                        ros::spinOnce();
                        break;

                }
                //Falls Rechtsauswahl 1 ist wird diese Bedingung wahr
            } else if (rechtsAuswahl == 2) {
                //Falls der Abstand zur Wand vor dem Roboter kleiner als 0,18 wird macht er eine Notbremsung und rechtsAuswahl wird auf 0 gesetzt, die Koordinaten zur Wegberechnung werden ebenfalls zurueckgesetzt
                if (averageVorne < 0.18) {  //Bewegungstyp und halter werden gesetzt um zu Puffern
                    ROS_INFO("NOTSTOPPER 2!!!!!!!!!!!!!!!!!!!!!!");
                    rechtsAuswahl = 0;
                    koordinatenVorwaerts = 0;
                    bewegungstyp = 0;
                    halter = 5;
                    aktualisierer = 2;
                    ros::spinOnce();
                }
                //Solange zureuckgelegte Distanz kleiner als 0,16 bzw -0,16 ist faehrt der Roboter geradeaus
                while (weg < 0.16 && weg > -0.16) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }
                //Sobald der Roboter den weg zurueckgelegt hat werden aktualisierer und halter gesetzt als Puffer, die koordinaten werden zurueckgesetzt, bewegungstyp und rechtsAuswahl werden beide auf 0 gesetzt
                aktualisierer = 2;
                koordinatenVorwaerts = 0;
                rechtsAuswahl = 0;
                bewegungstyp = 0;
                halter = 5;
                //rateH.sleep();
                ros::spinOnce();

            }
        }
        //Wenn Bewgungstyp 4 ist wird diese Bedingung erfuellt
        if (bewegungstyp == 4) {
            switch (Richtungsgeber) {
                //Richtungsgeber Switch zur aktuellen Position wegen Linksdrehung
                case 0:

                    //ROS_INFO("HALLO: %f", aktuelleRichtung);
                    //solange die Neunzig Grad Drehung nicht abgeschlossen ist, dreht sich der Roboter nach Links
                    while (aktuelleRichtung < 90.0) {
                        robot_move(NEUNZIG_LINKS);
                        ros::spinOnce();
                    }
                    //Sobald Drehung abgeschlossen wird der Richtungsgeber auf 90 gesetzt und bewegungstyp auf 0, sowie countercounter auf 2
                    Richtungsgeber = 90;
                    bewegungstyp = 0;
                    countercounter = 2;
                    ros::spinOnce();
                    break;

                case 90:
                    //Analog zu case 0
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
                    //Analog zu case 0
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
                    //Analog zu case 0
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
    //Halter wird zur Pufferung benutzt, solange Halter groeßer als 0 ist wird ros::spinOnce() aufgerufen, sowie ein sleep
    if (halter > 0) {
        ROS_INFO("Halter: %i", halter);
        halter--;
        rateH.sleep();
        ros::spinOnce();
        //Wenn der halter genau 0 ist, wird diese Bedingung erfuellt
    } else if (halter == 0) {

        //Falls counter 0 ist wird diese Bedingung erfuellt
        if (counter == 0) {//Ist für den Anfang damit der Turtlebot von der mitte aus zur ersten Wand fährt
            //Falls counter 0 ist wird diese Bedingung erfuellt
            if (countercounter == 0) {
                //Solange die Wand weiter entfernt ist als 0.19 faehrt der Roboter geradeaus
                while (averageVorne > 0.19) {
                    robot_move(GERADEAUS);
                    ros::spinOnce();
                }
                //sobald der Abstand zur Wand davor kleiner gleich 0.19 ist wird der countercounter auf 1 gesetzt
                if (averageVorne <= 0.19) {
                    countercounter = 1;
                }
            }
            //nachdem der countercounter 1 ist, wird nun der Bewegungstyp auf 4 gesetzt (und somit die Anfahrt an die erste Wand abgeschlossen indem Linksdrehung begonnen wird)
            if (countercounter == 1) {
                bewegungstyp = 4;
                ros::spinOnce();
            }
            //wenn countercounter 2 ist, wird der counter auf 1 gesetzt
            if (countercounter == 2) {
                counter = 1;
                ros::spinOnce();
            }

            //Wenn der counter 1 ist wird diese Bedingung erfuellt
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
