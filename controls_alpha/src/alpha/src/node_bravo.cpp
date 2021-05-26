//Stanley

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
FILE *fp = fopen("data_s.txt", "w");
//#include <vector>

typedef struct pointy{
    double x;
    double y;
} point;

typedef struct liney{
    point a;
    point b;
} line;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double c =  1;
double range = 0.5;
point front, rear;
const double pi = 2*acos(0.0);
int last_ind = 0;
double k = 0;

//std::vector<double> dists;
double min_dist;

//converts 2 points to line
line point_to_line(point a, point b){
    line L;
    L.a = a;
    L.b = b;
    
    return L;
}

//distance
double dist(point a, point b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
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
    //int size = 0;
    point pt;
    front.x = car_states->front.x;
    front.y = car_states->front.y;
    
    pt.x = path->poses[0].pose.position.x;
    pt.y = path->poses[0].pose.position.y;
    
    min_dist = dist(pt, front);
    
    //ROS_INFO("Size: %d\n", size);
    for(int i = last_ind; path->poses[i].pose.position.x != 0.0; i++){
        pt.x = path->poses[i].pose.position.x;
        pt.y = path->poses[i].pose.position.y;
        //size  = i;
        double d = dist(front, pt);
        if(min_dist >= d){
            min_dist = d;
            index = i;
        }
    }
    
    //ROS_INFO("Size: %d\n", size);
    if(index == -1){
        index = 0;
        ROS_INFO("No points found! :(\n");
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
    double kp = 0, kd = 0, ki = 0;
    
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

using namespace message_filters;

class sub_pub{
	public: 
		sub_pub(){
			pub = nh.advertise<prius_msgs::Control>("prius", 1000);
			
			sub1.subscribe(nh, "astroid_path", 1);
			sub2.subscribe(nh, "meteor_shower", 1);
			sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));   
			sync->registerCallback(boost::bind(&sub_pub::callback, this, _1, _2));

		}
		
        void callback(const nav_msgs::PathConstPtr& path, const alpha::StampedLinksConstPtr& car_states){
            int i = find_path_index(path, car_states);
            last_ind = i;
            prius_msgs::Control cmd;
            
            point front, pt, pt_nxt;
            front.x = car_states->front.x;
            front.y = car_states->front.y;
            
            rear.x = car_states->rear.x;
            rear.y = car_states->rear.y;
            
            pt.x = path->poses[i].pose.position.x;
            pt.y = path->poses[i].pose.position.y;
            
            pt_nxt.x = path->poses[i+1].pose.position.x;
            pt_nxt.y = path->poses[i+1].pose.position.y;
            
            line H_ = point_to_line(rear, front);
            line P_ = point_to_line(pt, pt_nxt);
            line L_ = point_to_line(front, pt);
            
            double sign = sgn(find_angle(H_, L_));
            
            double hdg_error = find_angle(H_, P_);
            
            double vel = get_vel(car_states);
            
            ros::param::get("/k_stan", k);
            ros::param::get("/c_stan", c);
            //ROS_INFO("HdgErr: %f, sign: %d, min_dist: %f, i: %d.", hdg_error, (int)sign, min_dist, i);
            
            double crs_term = sign*abs(atan((k*min_dist)/(vel + c)));
            //ROS_INFO("Cross track: %f", min_dist);
            ROS_INFO("hdg error: %f, crs_error: %f", hdg_error, sign*min_dist);
            
            double steer = hdg_error - crs_term;
            //ROS_INFO("index: %d, angle: %f", i, alpha*(180.0/pi));
            cmd.steer = (steer/35.0)*(180.0/pi);
            cmd.shift_gears = 2;
            
            double thr = pid_throttle(4.2, vel);
            
            cmd.throttle = (thr>0)? thr : 0;
            cmd.brake = (thr>0)? 0 : thr;
            //ROS_INFO("Throttle sent: %f", cmd.throttle);
            pub.publish(cmd);
            //ROS_INFO("published");
        }
	
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		
		
		message_filters::Subscriber<nav_msgs::Path> sub1;
		message_filters::Subscriber<alpha::StampedLinks> sub2;
		
		typedef sync_policies::ApproximateTime<nav_msgs::Path, alpha::StampedLinks> MySyncPolicy;
    		typedef Synchronizer<MySyncPolicy> Sync;
   		boost::shared_ptr<Sync> sync;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "controller_stanley");

	sub_pub brdcstr;
	
	ros::spin();
}

