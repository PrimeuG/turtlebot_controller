#include <ros/ros.h>
#include <turtlebot_controller/ServiceServer.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

bool fahren = true;                     //sorgt dafür das durch Starten des Servers der Turtlebot direkt anfängt zu fahren, die Clients geben dann jeweils einen anderen Bool zurück
float movex;
float movez;
double control_gain;


bool moven(turtlebot_controller::ServiceServer::Request &req, turtlebot_controller::ServiceServer::Response &res) {

    res.moveres = req.movereq;
    fahren = res.moveres;
    //ROS_INFO("REQUEST: %d", req.movereq);       zum kontrollieren was der Server gerade als "Status" hat, also was der Turtlebot gerade zu tun hat
    //ROS_INFO("RESPONSE: %d", res.moveres);

}

void CallbackLaser(const sensor_msgs::LaserScan::ConstPtr &msg) {    //In Zusammenarbeit mit Malte Kretschmann
    //ROS_INFO("LaserScan (min, max, angle)=(%f,%f)\n", msg->angle_min, msg->angle_max, msg->angle_increment);

    ros::NodeHandle nh1;
    ros::Publisher pub = nh1.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    float minimumRange = 3.5;
    int zaehler = 1;
    for (float i : msg->ranges) {
        if (i >= msg->range_min && i <= msg->range_max && i <= minimumRange) {
            minimumRange = i;
        } else zaehler += 1;

    }

    float angle = msg->angle_min + zaehler * msg->angle_increment;


    if (minimumRange >
        0.2) {    //In zusammenarbeit mit Florian Eimann       //wenn der Turtlebot noch weit entfernt ist, soll er sich justieren und rantasten bis er eine entfernung von <=0.2 hat
        if (angle > 6 && angle <
                         6.29) {                          //in dieser Bedingung wird geschaut bzw. entschieden ob der Turtlebot gerade aus fährt (in der Bedingung True)
            movex = control_gain;
            movez = 0;
        } else {                                                 //wenn in der IF Bedinung false als Wert kam, kommt der Turtlebot in diese Bedinung, die sagt, er solle sich so lange drehen bis er gerade vor dem Pillar steht
            movex = control_gain;
            movez = control_gain * (6.29 - angle);
        }
    } else {                                                                 //wenn der Turtlebot dicht genug ist, dann soll er sich zu dem Pillar mit der Geschwindigkeit x = 3 auf den Pillar zubewegen und diesen berühren
        movex = 0;
        movez = 0;
    }

}

int main(int argc, char **argv) {
    std::string scan_topic;
    int laserscan_queue;


    ros::init(argc, argv, "Service_Node");
    ros::NodeHandle n;
    n.param("turtlebot/queue_size", laserscan_queue, 1);
    n.param("turtlebot/control_gain", control_gain, 0.5);
    ros::Publisher geo = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::ServiceServer service = n.advertiseService("ServiceServer", moven);
    ros::Subscriber scan = n.subscribe("/scan", 10, CallbackLaser);

    n.param("turtlebot/control_gain", control_gain, 0.5);
    geometry_msgs::Twist msg;


    while (ros::ok()) {
        ros::spinOnce();
        if (fahren) {
            msg.linear.x = movex;
            msg.angular.z = movez;
            geo.publish(msg);
            ros::spinOnce();

        } else {
            msg.linear.x = 0;
            msg.angular.z = 0;
            geo.publish(msg);
            ros::spinOnce();
        }
    }
    //ros::spin();
}