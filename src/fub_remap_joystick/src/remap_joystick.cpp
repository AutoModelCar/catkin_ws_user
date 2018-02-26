#include <ros/ros.h>

#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>

namespace fub_remap_joystick {

class remap_joystick : public nodelet::Nodelet
{
public:
    remap_joystick() {}

    virtual ~remap_joystick() {}

	virtual void onInit() override
    {
        mWantedSpeedPublisher = getNodeHandle().advertise<std_msgs::Int16>("/manual_control/speed", 1);
        mWantedSteerAnglePublisher = getNodeHandle().advertise<std_msgs::Int16>("/manual_control/steering", 1);
        mGear=8;
        mInt=false;
        mSubscriberJoystic= getNodeHandle().subscribe("joy", 1, &remap_joystick::callbackJoy, this);

	}


private:
    void callbackJoy(sensor_msgs::Joy const & joy){

        mGear=mGear/(joy.buttons[4]+1);
        mGear=mGear*(joy.buttons[5]+1);
        if (mGear>8)
            mGear=8;
        if (mGear<4)
            mGear=4;

        if (joy.axes[2]!=0.0)
            mInt=true;
        if ((mGear==8)&&(mInt))
        	speedMsg.data=((joy.axes[2]+1.0)/2.0)*-250;
        else if (mInt)
        	speedMsg.data=((joy.axes[2]+1.0)/2.0)*250;
        else
            speedMsg.data=0;
        mWantedSpeedPublisher.publish(speedMsg);

        steerMsg.data = 90*joy.axes[0]+90;
        mWantedSteerAnglePublisher.publish(steerMsg);

    }
    /// subscriber
    ros::Subscriber mSubscriberJoystic;

    /// publisher
    ros::Publisher mWantedSpeedPublisher;
    ros::Publisher mWantedSteerAnglePublisher;

    double mGear;
    std_msgs::Int16 speedMsg;
    std_msgs::Int16 steerMsg;
    bool mInt;

};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fub_remap_joystick::remap_joystick, nodelet::Nodelet);
