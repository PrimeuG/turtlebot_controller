#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

//zurückgelegter Weg in zusammenarbeit mit Tom Cybart erstellt!!!
float previousX = 0; //aktuellste X Position
float rechnerx = 0; //erster X Wert
float previousy = 0;//aktuellste X Position
float rechnery = 0; //erster Y Wert
int counter = 0;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("LaserScan (min, angle)=(%f,%f)", msg->angle_min, msg->angle_increment);

    //Minimumrange Funktion von Malte Kretschmann übernommen
    float minimumRange = 1;
    for (float x : msg->ranges)  //FOR-Schleife zur Implementierung
        if (x >= msg->range_min && x <= msg->range_max && x <= minimumRange){ //wenn x größer gleich dem minimalen Winkelabstand und kleiner gleich dem maximalen winkelabstand
            minimumRange = x;                                                 //dann setze den minimalen Winkelabstand als x
        }
    ROS_INFO("Der Minimalabstand ist: %f ", minimumRange);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {


    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

    if (counter == 0) { //setzen des 1. Y und X Wertes
        if (msg->pose.pose.position.y < 0) {
            rechnery = -1 * (msg->pose.pose.position.y);
        } else { rechnery = msg->pose.pose.position.y; }
    }
    if (counter == 0) {
        if (msg->pose.pose.position.x < 0) {
            rechnerx = -1 * (msg->pose.pose.position.x);
        } else { rechnerx = msg->pose.pose.position.x; }
    }
    counter++;

    if (msg->pose.pose.position.x < 0) { //setzen des aktuellsten X und Y Wertes
        previousX = (msg->pose.pose.position.x) * -1;
    } else { previousX = (msg->pose.pose.position.x); }
    if (msg->pose.pose.position.y < 0) {
        previousy = (msg->pose.pose.position.y) * -1;
    } else { previousy = (msg->pose.pose.position.y); }

    float weg = sqrt(pow((previousX - rechnerx), 2) + pow((previousy - rechnery), 2)); //Formel zum berechnen der Entfernung zwischen 2 Punkten
    // ROS_INFO("erste Po X %f:", rechnerx);
    //  ROS_INFO("erste Po y %f:", rechnery);
    ROS_INFO("Der zurückgelegte Weg %f: ", weg); //Ausgabe des zurückgelegten Weges
}

int main(int argc, char **argv) {
    std::string scan_topic;
    int laserscan_queue;

    ros::init(argc, argv, "turtlebot_controller"); //
    ros::NodeHandle nodeHandle;

    nodeHandle.param<std::string>("turtlebot/topic_name", scan_topic, "/scan");
    nodeHandle.param("turtlebot/queue_size", laserscan_queue, 1000);

    ros::Subscriber odom = nodeHandle.subscribe("/odom", 1000, odomCallback);
    ros::Subscriber scan = nodeHandle.subscribe(scan_topic, laserscan_queue, laserCallback);

    printf("%s\n", scan_topic.c_str());
    printf("%i\n", laserscan_queue);

    if (!nodeHandle.getParam("turtlebot/topic_name", scan_topic)) { //testen ob der Parameter geladen wurde
        ROS_ERROR("Could not find parameter!");
    }

    //ros::Rate loop_rate(1);
    ros::spin();
    return 0;

}