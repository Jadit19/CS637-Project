#include <iostream>
#include <ros/ros.h>

#include <q_learning/data.h>

int RATE;
q_learning::data msg;

int main(int argc, char** argv){
    ros::init(argc, argv, "learner");
    ros::NodeHandle nh;

    if (!nh.getParam("rate", RATE)){
        ROS_ERROR("Couldn't load params!");
        return 1;
    }

    ros::Publisher pub_gains_ = nh.advertise<q_learning::data>("/learner", RATE);
    ros::Rate loopRate(RATE);

    msg.message.data = "Parameters post tuning";
    msg.kp = 48;
    msg.ki = 0.1;
    msg.kd = 100;

    while (ros::ok()){
        ros::spinOnce();

        pub_gains_.publish(msg);

        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}