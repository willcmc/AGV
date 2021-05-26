#include"ros/ros.h"
#include <nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>

void callback(const nav_msgs::PathConstPtr& path){
    double x = path->poses[0].pose.position.x;
    double y = path->poses[0].pose.position.y;
    
    ROS_INFO("%f, %f", x, y);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "find_pos1");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("astroid_path", 1000, callback);
    
    ros::spin();
}
