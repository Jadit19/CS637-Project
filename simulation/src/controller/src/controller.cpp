#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <q_learning/data.h>

int RATE = 30;

double kp = 0;
double ki = 0;
double kd = 0;

double altitude;
double targetAltitude = 5;
double baseVel = 150;
double angular_vel[6];

mav_msgs::Actuators motor_speed;

void setAngularVel(double vel) {
    for (int i=0; i<6; i++)
        angular_vel[i] = vel;
}

void setMotorSpeed(){
    double err = targetAltitude - altitude;
    double vel = kp * err * baseVel;
    setAngularVel(vel);
    std::cout << vel << std::endl;

    motor_speed.angular_velocities = angular_vel;
}

void gainsCallback(const q_learning::dataConstPtr& msg){
    kp = msg->kp;
    ki = msg->ki;
    kd = msg->kd;
    return;
}

void poseCallback(const geometry_msgs::PoseConstPtr& msg){
    altitude = msg->position.z;
    setMotorSpeed();
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber sub_gains_ = nh.subscribe<q_learning::data>("/learner", RATE, gainsCallback);
    ros::Subscriber sub_pose_ = nh.subscribe<geometry_msgs::Pose>("/firefly/ground_truth/pose", RATE, poseCallback);
    ros::Publisher pub_speed_ = nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", RATE);

    ros::Rate loopRate(RATE);

    while (ros::ok()){
        ros::spinOnce();

        pub_speed_.publish(motor_speed);

        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}