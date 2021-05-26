//Pure Pursuit

#include "ros/ros.h"
#include "prius_msgs/Control.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "alpha/StampedLinks.h"
#include <message_filters/subscriber.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cmath>
#include <stdio.h>
#include <std_msgs/Float64.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

typedef struct pointy{
    double x;
    double y;
} point;

typedef struct liney{
    point a;
    point b;
} line;

double look_ahead =  3;
double range = 0.5;
point rear, front;
const double pi = 2*acos(0.0);
int last_ind = 0, last_ind_closest = 0;
double min_dist;

FILE *fp = fopen("data_pp.txt", "w");

line point_to_line(point a, point b){
    line L;
    L.a = a;
    L.b = b;
    
    return L;
}

double dist(point a, point b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

bool vicinity(double num1, double num2){
    double l_bound = num1 - range;
    double u_bound = num1 + range;
    
    if(num2>l_bound && num2<u_bound) return true;     
    else return false;
}

double find_angle(line A, line B){
    double m1 = (A.a.y-A.b.y)/(A.a.x-A.b.x);
    double m2 = (B.a.y-B.b.y)/(B.a.x-B.b.x);
    
    double angle = atan(abs(m2-m1)/(1+m1*m2));
    if((m2-m1)<0) angle = -angle;
    return angle;
}
    
int find_path_index(const nav_msgs::PathConstPtr& path, const alpha::StampedLinksConstPtr& car_states){
    int index = -1;
    int size = 0;
    point pt;
    rear.x = car_states->rear.x;
    rear.y = car_states->rear.y;
    //ROS_INFO("Size: %d\n", size);
    for(int i = last_ind; path->poses[i].pose.position.x != 0.0; i++){
        pt.x = path->poses[i].pose.position.x;
        pt.y = path->poses[i].pose.position.y;
        size  = i;

        double d = dist(pt, rear);
        if(vicinity(d, look_ahead)){ 
            index = i;
            break;
        }
    }
    //ROS_INFO("Size: %d\n", size);
    if(index == -1){
        index = 0;
        ROS_INFO("Too far! I'm confused :(\n");
    }
    return index;
}

double get_vel(const alpha::StampedLinksConstPtr& car_states){
    double vx = car_states->vel.x;
    double vy = car_states->vel.y;
    double vel = sqrt(vx*vx+vy*vy);
    //ROS_INFO("Vel: %f", vel);
    return vel;
}

double pid_throttle(double set, double pres){
    double kp = 2.5, kd = 0, ki = 0;
    
    ros::param::get("/kp", kp);
    ros::param::get("/ki", ki);
    ros::param::get("/kd", kd);
    
    static double e_last = 0;
    static double e_sum = 0;
    double e = set - pres;
    double e_diff = abs(e-e_last);

    e_sum += e;
    
    if(e_diff>1) ki = 0;
    
    double throttle = 0;
    throttle = kp*e + ki*e_sum + kd*e_diff;
    
    e_last = e;
    return throttle;
}

int find_nearest(const nav_msgs::PathConstPtr& path, const alpha::StampedLinksConstPtr& car_states){
    int index = -1;
    //int size = 0;
    point pt;
    front.x = car_states->front.x;
    front.y = car_states->front.y;
    
    pt.x = path->poses[0].pose.position.x;
    pt.y = path->poses[0].pose.position.y;
    
    min_dist = dist(pt, front);
    
    //ROS_INFO("Size: %d\n", size);
    for(int i = last_ind_closest; path->poses[i].pose.position.x != 0.0; i++){
        pt.x = path->poses[i].pose.position.x;
        pt.y = path->poses[i].pose.position.y;
        //size  = i;
        double d = dist(front, pt);
        if(min_dist >= d){
            min_dist = d;
            index = i;
        }
    }
    return index;
}
using namespace message_filters;

class sub_pub{
	public: 
		sub_pub(){
			pub = nh.advertise<prius_msgs::Control>("prius", 1000);
			pub2 = nh.advertise<std_msgs::Float64>("errors", 1000);
			sub1.subscribe(nh, "astroid_path", 1);
			sub2.subscribe(nh, "meteor_shower", 1);
			sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));   
			sync->registerCallback(boost::bind(&sub_pub::callback, this, _1, _2));

		}
		
        void callback(const nav_msgs::PathConstPtr& path, const alpha::StampedLinksConstPtr& car_states){
            int i = find_path_index(path, car_states);
            last_ind = i;
            prius_msgs::Control cmd;
            std_msgs::Float64 msg;
            
            point front, pt;
            front.x = car_states->front.x;
            front.y = car_states->front.y;
            
            pt.x = path->poses[i].pose.position.x;
            pt.y = path->poses[i].pose.position.y;
            
            line H_ = point_to_line(rear, front);
            line L_ = point_to_line(rear, pt);
            
            line Z_ = point_to_line(front, pt);
            
            double sign = sgn(find_angle(H_, Z_));
            
            double alpha = find_angle(H_, L_);
            ros::param::get("/lookie", look_ahead);
            double steer = atan((2*1.45*sin(alpha))/look_ahead);
            //ROS_INFO("index: %d, angle: %f", i, alpha*(180.0/pi));
            cmd.steer = (steer/40.0)*(180.0/pi);
            cmd.shift_gears = 2;
            
            double vel = get_vel(car_states);
            
            double thr = pid_throttle(4.2, vel);
            
            cmd.throttle = (thr>0)? thr : 0;
            cmd.brake = (thr>0)? 0 : thr;
            //ROS_INFO("Throttle sent: %f", cmd.throttle);
            
            last_ind_closest = find_nearest(path, car_states);
            ROS_INFO("min_dist: %f, vel: %f", min_dist, vel);
            msg.data = min_dist*sign;
            pub2.publish(msg);
            fprintf(fp, "min_dist: %f, vel: %f\n", sign*min_dist, vel);
            
            pub.publish(cmd);
            //ROS_INFO("published");
        }
	
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Publisher pub2;
		
		message_filters:: Subscriber<nav_msgs::Path> sub1;
		message_filters::Subscriber<alpha::StampedLinks> sub2;
		
		typedef sync_policies::ApproximateTime<nav_msgs::Path, alpha::StampedLinks> MySyncPolicy;
    		typedef Synchronizer<MySyncPolicy> Sync;
   		boost::shared_ptr<Sync> sync;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "controller_pp");

	sub_pub brdcstr;
	
	ros::spin();
}

