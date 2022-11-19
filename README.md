# ROS2 Beginner Tutorials
This is a basic tutorial on writing and executing a Publisher-Subscriber pair using ROS2. Firstly, install all the dependencies required 

## **Setting up and sourcing ROS**
You will have to install ROS2 first, the instructions for which can be found on http://docs.ros.org/en/foxy/Installation.html. We will be using ROS2 Foxy for the purposes of this tutorial.

First, source your ROS2 environment:

```
source /opt/ros/foxy/setup.bash
```
Alternatively, you can add this to your system's .bashrc file to avoid sourcing ROS2 underlay each time a new terminal is opened: 
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

## **Setting up the project**
Now, create a workspace to work with new packages. This is done so that you don't have to build all the packages each time you try building a new package. 
```
mkdir -p <your_dir>/ros2_ws/src
cd ros2_ws/src/
```
Download the code from the repository:
```
git clone git@github.com:Bhargav-Soothram/beginner_tutorials.git
```
Now rename the package using (this is for build purposes)
```
mv beginner_tutorials cpp_pubsub
```

## **Building the package**
It is time to build the package! But before that, check if you have all the dependencies using resdep before going ahead:
```
rosdep install -i --from-path src --rosdistro foxy -y
```
Build the package:
```
cd ..
colcon build --packages-select cpp_pubsub
```

## **Running the package**
Open a new terminal, navigate to `ros2_ws`, and source the setup files:

### **Publisher and Subscriber**
```
. install/setup.bash
```
Start the talker node:
```
ros2 run beginner_tutorials talker
```
Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:
```
ros2 run beginner_tutorials listener
```
That is it, you have a publisher talking to a subscriber!

### **Service-Client**
Now, let us see how to request for a change in the string that is being published. This can be accomplished using a service-client (request-response) paradigm. To request for a change in the string,
```
ros2 service call /modify_string cpp_pubsub/srv/ModifyString "{new_string: <your_string>}"
```
## **Using Launch files**
We can use launch files to 'spin' multiple nodes at once with arguments passed to each of them. The arguments provided are the parameters to the nodes being executed. We have two parameters that can take user inputs,

* `my_message` *# the new message to be published*

* `my_message_freq` *# time between successive messages in ms*

To do this, execute the following command:
```
ros2 launch cpp_pubsub _launch.py my_message:=<your_message> my_message_freq:=<your_message_frequency>

``` 
You can choose to give one or both the arguments at launch.


## **Sample Outputs**
Attached are two sample outputs for different logger levels:
![Alt text](results/sample_outputs/ros2_rqt1.png?raw=true "INFO log level")
![Alt text](results/sample_outputs/ros2_rqt2.png?raw=true "ERROR log level")