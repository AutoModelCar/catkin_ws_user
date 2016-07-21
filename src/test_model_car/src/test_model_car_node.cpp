#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

class test_head
{
public:
	test_head(ros::NodeHandle nh)
	{
	    light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
	    head_subscriber = nh.subscribe("model_car/yaw", 1, &test_head::headCallback,this);
	}
	~test_head(){}

    void headCallback(const std_msgs::Float32& head)
	{
		if (head.data > 90.0 )
	    {
	    	if (light_command.data!="le")
	    	{
	    		light_command.data="le";
				ROS_INFO("head is bigger than 90");
				light_publisher.publish(light_command);
	    	}
	    }
	    else if (light_command.data!="ri")
	    {
	    	light_command.data="ri";
			ROS_INFO("head is less than 90");
			light_publisher.publish(light_command);
	    }     
	}
private:
    ros::Subscriber head_subscriber;
    ros::Publisher light_publisher;
    std_msgs::String light_command;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_model_car_node");
    ros::NodeHandle nh; 

    test_head test_head_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second

	while(ros::ok())
	{
		//rate.sleep();
		ros::spinOnce();
	}
    return 0;
}