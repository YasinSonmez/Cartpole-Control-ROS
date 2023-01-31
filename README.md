# Cartpole-Control-ROS

# Installation
1. Install [ROS](http://wiki.ros.org/ROS/Installation) (Noetic preffered)
2. Install [SFML](https://www.sfml-dev.org/) (For animations)
```
sudo apt-get install libsfml-dev
```
3. After the above preparation, please create and initialize a ROS workspace as below. We assume that your workspace is named catkin_ ws. Then, run the following commands to clone this repo and build it:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YasinSonmez/Cartpole-Control-ROS.git
cd ../
catkin_make
```
4. Add the "source" command to .bashrc to not source it in every new terminal window (Change the directory catkin_ws according to your folder name)
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

# Launch
1. Open a new terminal and start the ROS:
```
roscore
```
2. Open another terminal and start the controller of your choice:
```
rosrun controller pid_controller
rosrun controller lqr_controller
```
3. Open another terminal and start the simulation:
```
rosrun simulation simulation
```




# References
1. [https://github.com/jasleon/Inverted-Pendulum](https://github.com/jasleon/Inverted-Pendulum)
