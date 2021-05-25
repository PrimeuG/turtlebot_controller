#include "ros/ros.h"
#include "turtlebot_controller/ServiceServer.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>


float acceleration_x;
float acceleration_y;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {           // mit kleiner hilfe von Florian Eimann

    acceleration_x = msg->linear_acceleration.x;    //gibt dem float den aktuellen Wert der Acceleration
    acceleration_y = msg ->linear_acceleration.y;
}

int main(int argc, char **argv) {

    float crash; //dies ist die Variable die zeigt ab wann der kollisionsClient einen crash erwartet

    ros::init(argc, argv, "kollisionsclient");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<turtlebot_controller::ServiceServer>("ServiceServer");
    ros::Subscriber imu = n.subscribe("/imu", 1000, imuCallback);

    turtlebot_controller::ServiceServer srv;

    n.param<std::float_t>("turtlebot/crash", crash, -3);

    ros::Rate loop_rate(10000);


    while (ros::ok()) {
        if (crash > acceleration_x || crash > acceleration_y || crash < acceleration_y * -1) {                    //sofern acceleration_x kleiner als crash ist, bedeutet es das der Turtlebot einen crash hatte und somit der Turtlebot halten soll
            while (true) {                                  //diese Schleife besteht damit der Turtlebot nach einem crash nicht mehr weiter fährt
                srv.request.movereq = 0;                     // setzt den Request auf 0 -> false und somit gibt diese Zeile dem StartStopServer weiter das der Turtlebot anhalten soll
                ros::spinOnce();
                loop_rate.sleep();

                if (client.call(srv)) {         //Diese if Bedingung war mit hilfe von Florian Eimann, ohne diese stoppt mein Turtlebot nicht nach der kollision

                } else {

                    break;
                }

            }
        }
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}