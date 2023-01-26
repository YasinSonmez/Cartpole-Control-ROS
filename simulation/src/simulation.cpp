#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include <SFML/Graphics.hpp>
#include <iostream>
#include "inverted_pendulum.h"
#include <algorithm>

using namespace std;

class simulator
{
public:
    simulator()
    {
        // Topic you want to publish
        pub_ = n_.advertise<std_msgs::Float64MultiArray>("/state", 1);

        // Topic you want to subscribe
        sub_ = n_.subscribe("/controller_outputs", 1, &simulator::callback, this);

        window.create(sf::VideoMode(640, 480), "Inverted Pendulum");

        // Set initial conditions
        const double p_0 = 0;
        const double theta_0 = -5 * 3.141 / 180.0;
        Eigen::VectorXd x_0(4);
        x_0 << p_0, theta_0, 0, 0;

        // Create a model with default parameters
        ptr = new InvertedPendulum(x_0);
        ptr->Linearize();

        // Load font
        sf::Font font;
        if (!font.loadFromFile("Roboto-Regular.ttf"))
        {
            std::cout << "Failed to load font!\n";
        }

        // Create text to display simulation time
        text.setFont(font);
        text.setCharacterSize(24);
        const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
        text.setFillColor(grey);
        text.setPosition(480.0F, 360.0F);

        // Create text to display controller type
        type.setFont(font);
        type.setCharacterSize(24);
        const sf::Color turquoise = sf::Color(0x06, 0xC2, 0xAC);
        type.setFillColor(turquoise);
        type.setPosition(480.0F, 384.0F);

        // Create a track for the cart
        track = sf::RectangleShape(sf::Vector2f(640.0F, 2.0F));
        track.setOrigin(320.0F, 1.0F);
        track.setPosition(320.0F, 240.0F);
        const sf::Color light_grey = sf::Color(0xAA, 0xAA, 0xAA);
        track.setFillColor(light_grey);

        // Create the cart of the inverted pendulum
        cart = sf::RectangleShape(sf::Vector2f(100.0F, 100.0F));
        cart.setOrigin(50.0F, 50.0F);
        cart.setPosition(320.0F, 240.0F);
        cart.setFillColor(sf::Color::Black);

        // Create the pole of the inverted pendulum
        pole = sf::RectangleShape(sf::Vector2f(20.0F, 200.0F));
        pole.setOrigin(10.0F, 200.0F);
        pole.setPosition(320.0F, 240.0F);
        pole.setRotation(-theta_0);
        const sf::Color brown = sf::Color(0xCC, 0x99, 0x66);
        pole.setFillColor(brown);
    }

    void callback(const std_msgs::Float64 &input)
    {
        // Record control input
        control = input.data;
        // Constrain the input value
        double max_control = 100.0;
        control = max(-max_control, min(control, max_control));
    }
    void main()
    {
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            }
        }

        // Update the simulation
        sf::Time elapsed = clock.getElapsedTime();
        const float time = elapsed.asSeconds();
        const std::string time_msg = std::to_string(time);
        text.setString("Time   " + time_msg.substr(0, time_msg.find('.') + 2));

        // TODO Change to reset after 15 seconds
        if (time < 15 || 1)
        {
            ptr->Update(time, control);
        }
        else
        {
            delete ptr;
            ptr = new InvertedPendulum(x_0);
            clock.restart();
        }
        // Get the state
        Eigen::VectorXd x = ptr->GetState();

        // Publish the state
        std_msgs::Float64MultiArray state_msg;
        for (auto x_elem : x)
            state_msg.data.push_back(x_elem);
        // Add time as a variable
        state_msg.data.push_back(time);
        pub_.publish(state_msg);

        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            }
        }

        // Update SFML drawings
        cart.setPosition(int(320.0F + 100 * x(0)) % 640, 240.0F);
        pole.setPosition(int(320.0F + 100 * x(0)) % 640, 240.0F);
        pole.setRotation(-x(1) * 180.0 / 3.141);

        window.clear(sf::Color::White);
        window.draw(track);
        window.draw(cart);
        window.draw(pole);
        // window.draw(text);
        // window.draw(type);
        window.display();
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int count = 0;
    double control = 0.0;
    Eigen::VectorXd x_0;
    sf::RenderWindow window;
    InvertedPendulum *ptr;
    sf::Event event;
    sf::Text text;
    sf::Text type;
    sf::RectangleShape track;
    sf::RectangleShape cart;
    sf::RectangleShape pole;
    // Create a clock to run the simulation
    sf::Clock clock;

}; // End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator");
    simulator simulatorObject;

    ros::Rate loop_rate(50);
    ros::Time::init();

    while (ros::ok())
    {
        simulatorObject.main();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}