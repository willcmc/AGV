/*

**Template for quickly setting up Approximate Time Synchronization of incoming messages and simulatneous sub-pub using the class method in C++
**Contributor: William E. R., IIT Kharagpur
**Written October 2020
**Uploaded May 2021

**IMPORTANT NOTES: 
**replace library-name with the message containing folder
**replace message-name with the message name
*/

#include "ros/ros.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <string>


using namespace message_filters;

class sub_pub{
	public: 
		sub_pub(){
		    //Use as many as are required
			pub = nh.advertise<library-name::message-name1>("outgoing-topic-1", buffer_size_in);
			pub2 = nh.advertise<library-name::message-name2>("outgoing-topic-2", buffer_size_in);
			
			//Use as many as are required
			sub1.subscribe(nh, "incoming-topic-1", buffer_size_out);
			sub2.subscribe(nh, "incoming-topic-2", buffer_size_out);
			
			//List the subscriber objects in this fashion:
			sync.reset(new Sync(MySyncPolicy(10), sub1, sub2)); 
			
			//Just like "_1" and "_2", put in a series from "_1" through "_i", if you have i subscribers
			sync->registerCallback(boost::bind(&sub_pub::callback, this, _1, _2));

		}
		
        void callback(formal argument list of incoming messages demanded){
            //Whatever you want to do here! Usually will be where you publish the messages
        }
	
	private:
		ros::NodeHandle nh;
		
		//Use as many as are required
		ros::Publisher pub;
		ros::Publisher pub2;
		
		//Use as many as are required
		message_filters::Subscriber<library-name::message-name1> sub1;
		message_filters::Subscriber<library-name::message-name2> sub2;
		
		//list all the message types subscribed to in a similar fashion
		typedef sync_policies::ApproximateTime<library-name::message-name1, library-name::message-name2> MySyncPolicy;
    		typedef Synchronizer<MySyncPolicy> Sync;
   		boost::shared_ptr<Sync> sync;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "node-name");
	sub_pub brdcstr;
	ros::spin();
}
