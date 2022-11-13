#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <q_learning/data.h>

// Paramas
int RATE;
double gravity;
double fireflyMass;
double targetAltitude;

double dt;
double kp;
double ki;
double kd;

double prevErr = targetAltitude;
double integralErr = 0;
double derivativeErr = 0;

mav_msgs::RollPitchYawrateThrust thrust;

void noThrust(){
    thrust.thrust.z = 0;
}

void setThrust(double altitude){
    double err = targetAltitude - altitude;
    integralErr += err * dt;
    derivativeErr = (err-prevErr) / dt;

    double proportionalThrust = kp * err;
    double integralThrust = ki * integralErr;
    double derivativeThrust = kd * derivativeErr;
    
    thrust.thrust.z = proportionalThrust + integralThrust + derivativeThrust;
    thrust.thrust.z += fireflyMass * gravity;
    prevErr = err;
    return;
}

void gainsCallback(const q_learning::dataConstPtr& msg){
    kp = msg->kp;
    ki = msg->ki;
    kd = msg->kd;
    return;
}

void poseCallback(const geometry_msgs::PoseConstPtr& msg){
    setThrust(msg->position.z);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    if (!nh.getParam("rate", RATE) || !nh.getParam("mass", fireflyMass) || !nh.getParam("gravity", gravity) || !nh.getParam("altitude", targetAltitude)){
        ROS_ERROR("Couldn't load params!");
        return 1;
    }
    dt = 1.0 / double(RATE);

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