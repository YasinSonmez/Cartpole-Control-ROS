#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "pid.h"

using namespace std;

class controller
{
public:
    controller()
    {
        pub_ = n_.advertise<std_msgs::Float64>("/controller_outputs", 1);
        sub_ = n_.subscribe("/state", 1, &controller::callback, this);

        // Initialize PID controller
        c_ptr->Init(kp, ki, kd);
    }

    void callback(const std_msgs::Float64MultiArray &input)
    {
        // Calculate the input
        double angle = input.data[1];
        double time = input.data[4];
        double error = 0.0F - angle;
        c_ptr->UpdateError(time, error);
        double control = c_ptr->TotalError();

        // Publish the message
        std_msgs::Float64 msg;
        msg.data = control;
        pub_.publish(msg);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    // Set PID constants
    const double kp = 100.0F;
    const double ki = 100.0F;
    const double kd = 10.0F;
    PID *c_ptr = new PID();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    controller controllerObject;

    ros::spin();

    return 0;
}