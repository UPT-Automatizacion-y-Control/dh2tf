#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dh2tf/DHParameters.h>

float x=0, y=0, z=0;
tf::Quaternion q;
std::string tf_begin, tf_end;

void timerCallback(const ros::TimerEvent&)
{ 
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x,y,z) );
	transform.setRotation(q);
	ros::Time curr_stamp = ros::Time::now();
    br.sendTransform(tf::StampedTransform(transform, curr_stamp, tf_begin, tf_end)); 
}

void subCallback(const dh2tf::DHParameters::ConstPtr& msg)
{
	x = msg->a*cos(msg->theta);
	y = msg->a*sin(msg->theta);
	z = msg->d;
	tf::Matrix3x3 m(tfScalar( cos(msg->theta)), 
	                tfScalar(-cos(msg->alpha)*sin(msg->theta)),
	                tfScalar( sin(msg->alpha)*sin(msg->theta)),
	                tfScalar( sin(msg->theta)), 
	                tfScalar( cos(msg->alpha)*cos(msg->theta)),
	                tfScalar(-sin(msg->alpha)*cos(msg->theta)), 
	                tfScalar( 0.0),
	                tfScalar( sin(msg->alpha)), 
	                tfScalar( cos(msg->alpha)) );
	m.getRotation(q);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dh2tf");
  ros::NodeHandle nh("~"); 
  ROS_INFO("dh2tf"); 
  
  nh.param<std::string>("tf_begin", tf_begin, "map");
  nh.param<std::string>("tf_end"  , tf_end,   "");
  
  if (tf_end == "")
  {
    ROS_ERROR("tf_end is not defined!!");
    return -1;
  }
  else
  	ROS_INFO("Will publish tf from %s to %s using DH parameters",tf_begin.c_str(),tf_end.c_str());
  	
  q.setRPY(0,0,0);
  
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/10.0), timerCallback); 
  ros::Subscriber sub = nh.subscribe("dh", 1000, subCallback);  
  ros::spin();
}
