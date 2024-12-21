#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include <cmath>
struct cmd_vel {
    float lx = 0.0;
    float ly = 0.0;
    float lz = 0.0;

    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;

    ros::Time last_update_time; // Last time angular velocity was updated
    ros::Time interpolation_start_time; // Time when interpolation starts
    bool angular_interpolating = false; // Flag to indicate interpolation
    bool waiting_to_interpolate = false; // Flag to indicate waiting period
    float angular_start = 0.0; // Starting value for interpolation
    float interpolation_duration = 0.8; // Duration for interpolation (in seconds)
    float wait_duration = 0.3; // Time to wait before starting interpolation (in seconds)

    float radius = 0.16;

    void increaseLinear() {
        lx += 0.01;
    }

    void decreaseLinear() {
        lx -= 0.01;
    }

    void increaseAngular() {
        az += 0.1;
        updateAngularState();
    }

    void decreaseAngular() {
        az -= 0.1;
        updateAngularState();
    }

    void reset() {
        lx = 0.0;
        ly = 0.0;
        lz = 0.0;

        ax = 0.0;
        ay = 0.0;
        az = 0.0;

        angular_interpolating = false;
        waiting_to_interpolate = false;
    }

    void updateAngularState() {
        last_update_time = ros::Time::now();
        angular_interpolating = false; // Stop interpolation if active
        waiting_to_interpolate = true; // Set waiting state
    }

    void updateAngular() {
        ros::Time current_time = ros::Time::now();
        ros::Duration time_since_update = current_time - last_update_time;

        if (waiting_to_interpolate) {
            if (time_since_update.toSec() >= wait_duration) {
                // Start interpolation after waiting period
                angular_start = az;
                interpolation_start_time = current_time;
                angular_interpolating = true;
                waiting_to_interpolate = false;
            }
        }

        if (angular_interpolating) {
            ros::Duration time_since_start = current_time - interpolation_start_time;
            float t = time_since_start.toSec() / interpolation_duration;

            if (t >= 1.0) {
                az = 0.0; // Stop interpolation after duration
                angular_interpolating = false;
            } else {
                az = angular_start * (1.0 - t); // Linearly interpolate towards zero
            }
        }
    }

    float getVelocity(){
        float linear_velocity_rotational = az * radius;
        float tVelocity = sqrt((pow(lx, 2) + pow(linear_velocity_rotational, 2)));
        
        return std::round(tVelocity * 100.0f) / 100.0f;
    }
};

// Declare a global instance of cmd_vel
struct cmd_vel movedata;

void debug() {
    ROS_INFO_STREAM("Publishing Twist message: linear.x = " << movedata.lx
                    << ", angular.z = " << movedata.az);
}

// Callback function for subscriber
void callback(const std_msgs::Int32::ConstPtr& msg) {
    switch (msg->data) {
        case 1:
            movedata.increaseLinear();
            break;
        case 2:
            movedata.decreaseLinear();
            break;
        case 3:
            movedata.decreaseAngular();
            break;
        case 4:
            movedata.increaseAngular();
            break;
        case 5:
            movedata.reset();
            break;
        default:
            ROS_WARN_STREAM("Invalid command received: " << msg->data);
            break;
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "movementController");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("manual_controller", 1000, callback);

    ros::Publisher speedPub = n.advertise<std_msgs::Float32>("speed_value", 10);

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate loop_rate(10); 

    std::cout << "Start motor controller" << std::endl;

    while (ros::ok()) {
        ros::spinOnce();

        // Update angular interpolation
        movedata.updateAngular();

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = movedata.lx;
        twist_msg.linear.y = movedata.ly;
        twist_msg.linear.z = movedata.lz;

        twist_msg.angular.x = movedata.ax;
        twist_msg.angular.y = movedata.ay;
        twist_msg.angular.z = movedata.az;

        pub.publish(twist_msg);
        std_msgs::Float32 vel;
        vel.data = movedata.getVelocity();
        speedPub.publish(vel);
        //debug();
        loop_rate.sleep();
    }

    return 0;
}
