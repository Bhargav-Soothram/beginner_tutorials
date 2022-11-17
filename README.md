# ROS2 Beginner Tutorials - Publisher/Subscriber
This is a basic tutorial on writing and executing a Publisher-Subscriber pair using ROS2. Firstly, install all the dependencies required 

You will have to install ROS2 first, the instructions for which can be found on http://docs.ros.org/en/foxy/Installation.html. We will be using ROS2 Foxy for the purposes of this tutorial.

First, source your ROS2 environment:

```
source /opt/ros/foxy/setup.bash
```
Alternatively, you can add this to your system's .bashrc file to avoid sourcing ROS2 underlay each time a new terminal is opened: 
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

Now, create a workspace to work with new packages. This is done so that you don't have to build all the packages each time you try building a new package. 
```
mkdir -p <your_dir>/ros2_ws/src
cd ros2_ws/src/
```
Download the code from the repository:
```
git clone git@github.com:Bhargav-Soothram/beginner_tutorials.git
```
It is time to build the package! But before that, check if you have all the dependencies using resdep before going ahead:
```
rosdep install -i --from-path src --rosdistro foxy -y
```
Build the package:
```
cd ..
colcon build --packages-select beginner_tutorials
```
Open a new terminal, navigate to `ros2_ws`, and source the setup files:
```
. install/setup.bash
```
Now run the talker node:
```
ros2 run beginner_tutorials talker
```
Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:
```
ros2 run beginner_tutorials listener
```
That is it, you have a publisher talking to a subscriber!