#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "lqr.h"

using namespace std;

class controller
{
public:
    controller()
    {
        pub_ = n_.advertise<std_msgs::Float64>("/controller_outputs", 1);
        sub_ = n_.subscribe("/state", 1, &controller::callback, this);

        // System parameters

        // Calculate LQR parameters
        double M_ = 1.0;
        double m_ = 1.0;
        double J_ = 1.0;
        double l_ = 1.0;
        double c_ = 1.0;
        double gamma_ = 1.0;
        double g_ = 9.81;

        double M_t_ = M_ + m_;
        double J_t_ = J_ + m_ * std::pow(l_, 2);
        double mu = M_t_ * J_t_ - std::pow((m_ * l_), 2);

        Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(4, 4);
        A_(0, 2) = 1;
        A_(1, 3) = 1;
        A_(2, 1) = std::pow((m_ * l_), 2) * g_ / mu;
        A_(2, 2) = -c_ * J_t_ / mu;
        A_(2, 3) = -gamma_ * l_ * m_ / mu;
        A_(3, 1) = M_t_ * m_ * g_ * l_ / mu;
        A_(3, 2) = -c_ * l_ * m_ / mu;
        A_(3, 3) = -gamma_ * M_t_ / mu;

        Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(4, 1);
        B_(2, 0) = J_t_ / mu;
        B_(3, 0) = l_ * m_ / mu;

        optimal.A_ = A_;
        optimal.B_ = B_;
        optimal.Q_ = Eigen::MatrixXd::Identity(4, 4);
        optimal.Q_(0, 0) = 10;
        optimal.R_ = Eigen::MatrixXd::Identity(1, 1);
        optimal.Compute();
    }

    void callback(const std_msgs::Float64MultiArray &input)
    {
        // Convert message to state as eigen vector
        Eigen::VectorXd state(4);
        state << input.data[0], input.data[1], input.data[2], input.data[3];
        // Calculate the optimal control
        double control = optimal.Control(state)(0, 0);

        // Publish the message
        std_msgs::Float64 msg;
        msg.data = control;
        pub_.publish(msg);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    // Design LQR controller
    LQR optimal;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    controller controllerObject;

    ros::spin();

    return 0;
}