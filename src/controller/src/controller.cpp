#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <q_learning/data.h>

int RATE = 30;

double kp = 0;
double ki = 0;
double kd = 0;
double fireflyMass = 1.5;
double gravity = 9.8;

double altitude;
double targetAltitude = 2;
std::vector<double> angularVel(6);

mav_msgs::RollPitchYawrateThrust thrust;

void noThrust(){
    thrust.thrust.z = 0;
}

void setThrust(){
    double err = targetAltitude - altitude;
    ROS_INFO("Error: %lf\n", err);
    double proportionalThrust = kp * err;
    thrust.thrust.z = fireflyMass*gravity + proportionalThrust;
}

void gainsCallback(const q_learning::dataConstPtr& msg){
    kp = msg->kp;
    ki = msg->ki;
    kd = msg->kd;
    return;
}

void poseCallback(const geometry_msgs::PoseConstPtr& msg){
    altitude = msg->position.z;
    ROS_INFO("Altitude: %lf", altitude);
    setThrust();
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber sub_gains_ = nh.subscribe<q_learning::data>("/learner", RATE, gainsCallback);
    ros::Subscriber sub_pose_ = nh.subscribe<geometry_msgs::Pose>("/firefly/ground_truth/pose", RATE, poseCallback);
    ros::Publisher pub_thrust_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", RATE);
    ros::Rate loopRate(RATE);

    noThrust();

    while (ros::ok()){
        ros::spinOnce();

        pub_thrust_.publish(thrust);

        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}