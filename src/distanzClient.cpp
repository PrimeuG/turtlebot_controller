#include <ros/ros.h>
#include <cstdlib>
#include <turtlebot_controller/ServiceServer.h>
#include <sensor_msgs/LaserScan.h>

float uebergeber = 3.5;                         //ist die minimumRange als Startwert, da sonst der Turtlebot nicht losfährt, da scheinbar kein Messwert bedeutet, es wäre zu klein der Abstand

void CallbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg){            // Diese Callback Funktion habe ich aus meinen Aufgaben davor genommen
    float minimumRange = 3.5;
    int zaehler = 1;
    for (float i : msg->ranges) {
        if (i >= msg->range_min && i <= msg->range_max && i <= minimumRange){
            minimumRange=i;
        } else zaehler += 1;

    }

    uebergeber = minimumRange;

}

int main(int argc, char **argv){

    float mindistanz;

    ros::init(argc, argv, "turtlebot_controller_client");


    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlebot_controller::ServiceServer>("ServiceServer");
    turtlebot_controller::ServiceServer service;
    nh.param<std::float_t>("turtlebot/mindistanz", mindistanz, 0.7);
    ros::Subscriber scan = nh.subscribe("/scan", 1, CallbackLaser);

    ros::spinOnce();
    while(ros::ok()) {
        ros::spinOnce();
        ROS_INFO("uebergeber %f", uebergeber);
        if (uebergeber < mindistanz) {

            while(true){                                                    //da die Bremse ein halt sein soll, habe ich diese while Schleife hinzugefügt, das der Turtlebot nicht weiter fährt, sofern man den Gegenstand von ihm enfternt
                service.request.movereq = 0;
                ros::spinOnce();
                if (client.call(service)) {
                    ROS_INFO("bool: %ld", service.response.moveres);
                }
            }
        }
    }

    return 0;
}