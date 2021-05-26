#include"ros/ros.h"
#include"alpha/StampedLinks.h"
#include<geometry_msgs/Point.h>
#include <gazebo_msgs/LinkStates.h>

ros::Publisher pub;

void callback(const gazebo_msgs::LinkStatesConstPtr& car_states){
    alpha::StampedLinks new_msg;
    
    new_msg.front.x = car_states->pose[38].position.x;
    new_msg.front.y = car_states->pose[38].position.y;
    
    new_msg.rear.x = car_states->pose[37].position.x;
    new_msg.rear.y = car_states->pose[37].position.y;
    
    new_msg.vel.x = car_states->twist[37].linear.x;
    new_msg.vel.y = car_states->twist[37].linear.y;
    
    new_msg.header.stamp = ros::Time::now();
    pub.publish(new_msg);
    //ROS_INFO("published");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "stamper");
    ros::NodeHandle n;
    pub = n.advertise<alpha::StampedLinks>("meteor_shower", 1000);
    ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, callback);

    ros::spin();
}
