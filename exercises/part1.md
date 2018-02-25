# Workshop "ROS for Engineers" - part 1

author (dutch): Eric Dortmans (e.dortmans@fontys.nl)

translation (english): Kris Piters (k.piters@fontys.nl)


## Helpful tips before you start

You can open a new terminal window with the key combination: *CTRL-ALT-T*.

All commands support *tab-completion*, that is, as soon as you press the *TAB* key, Ubuntu or ROS tries to complete what you have already typed.

You can walk through previously entered commands using the *arrow keys*. A previously entered command can be modified and / or re-executed by pressing the *ENTER* key.

You can stop a program by tapping *CTRL-C* in the relevant terminal window. You can also close the entire terminal window.

Some commonly used Linux commands if you work with ROS:

- *cd* or *cd ~* (go to your home directory)
- *cd ~/catkin_ws* (go to the top of your workspace, here you have to do *catkin_make* to build your ros packages)
- *cd ~/catkin_ws/src* (go to the src directory of your workspace, here are your packages)

## ROS installation & update

If all goes well, you have the [Kinetic version of ROS installed in Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu). Open a new terminal window and enter the following command to check your ROS version:

    rosversion -d
    
Update Ubuntu and ROS as follows:

    sudo apt-get update && sudo apt-get -y upgrade

Now install `git`, a version management tool and for downloading ROS projects; [GitHub](https://github.com/):

    sudo apt-get -y install git

You are now ready to start your ROS exercises.

## Creating a ROS workspace

When installing ROS you'll automatically have a nice number of standard packages installed. Often you want to create new packages yourself or reuse packages by others. You therefore need a so-called *workspace* where you can create or download and build those packages

Create a [ROS workspace](http://wiki.ros.org/catkin/workspaces#Catkin_Workspaces), for example *catkin_ws* (but the name of the workspace is free to choose). If you don't have a workspace yet, you can create one as follows (example uses the default catkin_ws workspace name):
```
source /opt/ros/kinetic/setup.bash
    
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
Make this workspace your default workspace (because you can have multiple workspaces):

    source ~/catkin_ws/devel/setup.bash

To have this command executed automatically in every new terminal window you open, you can add it to your *.bashrc* file as follows (It is in your home directory):

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

If you want to know the contents of your *.bashrc* file you can print the file to your terminal screen:

    cat ~/.bashrc

You can also open this file in a text editor (Ubuntu default: gedit):

    gedit ~/.bashrc

This way you can see how the ROS environment variables are set:

    env | grep ROS
    
## Download and install ROS packages

There are also a lot of standard packages installed when installing ROS. One of them is *turtlesim*, a 2D simulation package, made for educational purposes. Let's see if that package is indeed available.

Is the package `turtlesim` installed?

    rosversion turtlesim

Where is it installed?

    rospack find turtlesim

Many packages aren't installed by default. They are available as a binary download (so-called debian packages).
Which packages are available to extend your ROS installation?

    apt-cache search ros-kinetic-

There are a lot! You can find an overview [here](http://www.ros.org/browse/list.php).

```
NOT AVAILABLE FOR ROS KINETIC
TODO: change to different package for this tutorial

Let's download and install the package *arbotix_python*. We are not interested in the sources, but only want to install and use the package. Let's see if the relevant ROS package is available as a Debian (.deb) installation package. Which debian packages are there with the text *arbotix* in their name?


    apt-cache search ros-ki- | grep arbotix


Apparently the relevant Debian package is called *ros-indigo-arbotix-python*. Now that we know this, installing is a breeze:

    sudo apt-get install ros-indigo-arbotix-python

Check whether it is successful:

    rospack find arbotix_python

As you can see, this package has been added to the ROS installation and not to your workspace.

Sometimes we want to download ROS packages that are only available in source form, or whose sources we want to view and perhaps modify. We do not want to add these packages in binary form to our ROS installation, but download them in source form in our workspace.
```

Let's download some helpful ROS sample packages from GitHub to your workspace.
```
    cd ~/catkin_ws/src
    git clone https://github.com/dortmans/ros_examples.git
    cd ~/catkin_ws
    catkin_make
```
If all goes well, ROS can find those packages.

    rospack find agitr

You can also go to the directory of the package using [roscd](http://wiki.ros.org/rosbash#roscd), for example to make changes:

    roscd agitr
    
## Nodes, topics, messages

Start the node *listener* from the package *beginner_tutorials*

    rosrun beginner_tutorials listener

Why doesn't it work? How can you solve that?

Open a new terminal window (ctrl-alt-t) and enter the following command:

    rosnode

Running the [rosnode](http://wiki.ros.org/rosnode) command without arguments will list all the possible arguments for the command.

Check which nodes are running:

    rosnode list

Which topics have been created?

    rostopic list

Take a look at the ROS Graph:

    rqt_graph

Now we are going look at topics using the [rostopic](http://wiki.ros.org/rostopic) command:

    rostopic

To which topic is the node *listener* subscribed, i.e. to which topic is it listening?

What type of message can be published on this topic?

    rostopic type *topic_name*

What does such a message type consist of?

    rosmsg show *message_type*

In Linux you can pass the output of a command to another command by means of a *pipe* sign (|):

    rostopic type *topic_name* | rosmsg show

Try to manually publish a message to this topic:

    rostopic pub *topic_name* *message_type* *message_content*

Tip: use the *TAB* key after typing the message type.

What happened in the terminal where you started the *listener* node?

ROS is made to control robots and not to make chat programs. In practice, messages are more complex and nodes will advertise and publish on several topics. Moreover, nodes often have parameters and can be controlled via services in addition to messages via topics. Now let's take a look at a node that a little bit more complex: a 2D robot simulator.

Start the node *turtlesim_node* from the package *turtlesim*

- Which nodes are running?

- Which topics have been created?

Analyse the ROS Computation Graph.

- To which topics is the node *turtlesim* subscribed? To which topics does it publish?

- What messagetype is published on the topic */turtle1/pose* ?

With the rostopic command you can do much more. Use this command to address the following questions:

- At which frequency (rate) are messages published to this topic?

Analyze the messages that are currently being published on this topic.

- What do you think the published data means?

- Which message type is associated with the topic */turtle1/cmd_vel*?

- What happens when using the following command?
```
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}}"
```    
And what about this command?
```
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{angular: {z: 1.57}}"
```
Draw a square with the turtle (approximately).

- What happens when you execute the following command?
```
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
```
- How does it compare to next command?
```    
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
So far we have only used one application with only one node. In practice, an application will consist of several nodes. We have moved the turtle using ROS commands from the terminal. That is enough to test the node, but of course it is not an integrated application. Suppose we want to control the turtle with the arrow keys of the keyboard. For this we will use another node that will convert input from the keyboard into control commands for the turtle.

Use a new terminal to start the *turtle_teleop_key* node from the *turtlesim* package. You will probably know how to do that by now.

Use the computation graph to see how the *turtle_teleop_key* node is linked to the *turtlesim_node*:

Try using *rostopic echo* to analyze the messages that are being published by *turtle_teleop_key* when you press a certain arrow key.

## Services

Topics arent the only way for nodes to exchange information. A node can also call a service offered by another node (or offer a service itself). We can test a service from a terminal by using the [rosservice](http://wiki.ros.org/rosservice) command.

Which services are offered by the *turtlesim* node? 

    rosservice list

What are the arguments for the set_pen service?

    rosservice args /turtle1/set_pen

For more detail:

    rosservice type /turtle1/set_pen | rossrv show

Call the set_pen service (using *rosservice call*) to change the color of the pen to red.

Move the turtle to see if it has worked.

Disable the pen; move the turtle to see if it worked.

Reset turtlesim:

    rosservice call /reset

## Parameters

Nodes can also read parameters. These parameters have to be published before starting the node, this usually happens by using startup scripts ([launch files](http://wiki.ros.org/roslaunch)).

Which parameters are currently defined?

    rosparam list

Dump all the parameters to a file in YAML format:

    rosparam dump dump.yaml

Let's take a look at that file:

    cat dump.yaml

Change the value of the parameter */background_b*
   
    rosparam set /background_b 0

Clear the turtlesim node so the parameters will be reread:

    rosservice call /clear

Add a new turtle to the sim:
   
    rosservice call /spawn 2 2 1.57 "r2d2"

Which topics are available after doing this?

    rostopic list

Make the newly added turtle move.

## Recording and playing back messages

Published messages can be recorded to a *bag* file. They can be played back at a later time. This is very useful for testing!

Let's start recording messages on the *turtle1/cmd_vel* topic using [rosbag](http://wiki.ros.org/rosbag):

    rosbag record -O /tmp/turtle1 /turtle1/cmd_vel

Make *turtle1* move. 

To stop recording hit *CTRL-C* in the terminal running rosbag.

Reset *turtlesim* to start with a clean slate:

    rosservice call /reset
    
Play back the recorded messages stored in the *bag file*:

    rosbag play /tmp/turtle1.bag

What happens?

## Launch files

Up until now we had to start each node individually from a new terminal window (and *roscore* of course). Starting your nodes this way can be an inconvenience. Luckily ROS has a mechanism to start a whole collection of nodes at once: [roslaunch](http://wiki.ros.org/roslaunch). Your collection of nodes (and their associated parameters) have to be specified for use with the *roslaunch* command. Such a specification is called a *launch file*.

Analyse the content of the launch files in the *agitr* package. The folder containing the launch files can be opened with Ubuntu's filemanager (*nautilus*) using the following command:

    nautilus `rospack find agitr`/launch
    
Try to launch them with the *roslaunch* command:

    roslaunch agitr <launchfile_name>


Next we are going to make a useful launch file to launch the nodes needed to show a realtime image from a webcam on our screen.

Install a USB camera driver package ([usb_cam](http://wiki.ros.org/usb_cam)):

    sudo apt-get -y install ros-kinetic-usb-cam 

This package contains a node for controlling the webcam.

We also need a node that is able to read the images and show them on a screen. for this we will use the *image_view* node included in the [image_view](http://wiki.ros.org/image_view) package. This package should be installed by default.

We want to be able to simultaneously start both nodes with their required parameters. Let's make a launch file to help us achieve this goal. First we'll create a new package that will contain our launch file. Let's call it *usb_camera*.

    cd ~/catkin_ws/src
    catkin_create_pkg usb_camera std_msgs rospy roscpp
    cd ~/catkin_ws
    catkin_make

Let's check if ROS can find our new package:

    rospack find usb_camera

In our still empty package we'll make a directory named *launch*. This directory will contain our launch file.

    roscd usb_camera
    mkdir launch
    
It's probably useful to open this directory with *Nautilus*:

    nautilus `rospack find usb_camera`/launch
    
Create a new file named *usb_camera.launch* in the *launch* directory and give it the following content:

    <launch>
      <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
        <param name="video_device" value="/dev/video0" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <!--<param name="pixel_format" value="mjpeg" /> -->
        <param name="pixel_format" value="yuyv" />
        <param name="camera_frame_id" value="usb_cam" />
        <param name="io_method" value="mmap"/>
      </node>
      <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
        <!-- <remap from="image" to="/usb_cam/image_raw"/> -->
        <param name="autosize" value="true" />
      </node>
    </launch>

PS: usually the webcam device in Ubuntu is registered as */dev/video0*, but might also be */dev/video1*.  

PPS: not every webcam is the same. You might have to change the *pixel_format* depending on the camera.

Test the launchfile:

    roslaunch usb_camera usb_camera.launch

Analyze the computation graph:

    rqt_graph

What structural problem do you see? Hint: play around with rqt_graph's settings to get more information in your graph.

Solve it by adjusting the launch file. If you succeed you can look at yourself nicely ;-)


## rqt

Many things you have done so far (using terminal commands) can also be done using the software [*rqt*](http://wiki.ros.org/rqt). This program provides a graphical user interface (GUI) and can be used to visualize and control many things in the ROS system (messages for example). 

Start the *rqt* GUI:

    rqt

Explore the different plugins.

Some plugins you already know (rqt_graph, rqt_image_view), we have started and used them before. Within rqt all of these plugins are nicely integrated into one interface.

## Controlling a real mobile robot

Up until now all nodes have been run on our own laptop. Usually a part of the ROS nodes run on the robot itself. The nodes on our laptop will have to use a network connection (ea. Wifi) to communicate with the nodes running on the robot.

ROS is designed to operate in a distributed way. For this to work you need to know on which machine the ROS Master node (roscore) is running. Usually it'll run on the robot itself. It is sufficient to set the ROS environment variables for ROS_IP and ROS_MASTER_URI to enable communication between your laptop and the robot. ROS_IP (on your laptop) has to be set to the ip address of your laptop. 

Make sure the robot and your laptop are both connected to the same network. Then use the following command to view your ip addresses:

    hostname -I

ROS_IP will have to be set to the address that partly matches the robot's address (eg 192.168.x.x). If only one address is returned you can simply reuse the hostname command:

    export ROS_IP=`hostname -I`

If not, you'll have to set the ip address manually:

    export ROS_IP=<IP_address_of_your_laptop>

Next you'll have to set the ROS_MASTER_URI value. This will allow the nodes running on your laptop to access to the ROS Master on the robot.

    export ROS_MASTER_URI=http://<ip_address_of_robot>:11311

These exports can also be added to your *.bashrc* file, this way they are automatically executed when opening a new terminal window.  

    gedit ~/.bashrc

Check if you can communicate with the ROS Master (now running on the robot):

    rostopic list

Controlling the robot works similary to the simulated robot in turtlesim.

Make the robot drive around in a circle using the *rostopic pub* command.

## References
- [A Gentle Introduction to ROS](http://www.cse.sc.edu/~jokane/agitr/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [YAML on the ROS command line](http://wiki.ros.org/ROS/YAMLCommandLine)
- [rosparam](http://wiki.ros.org/rosparam)
- [roslaunch](http://wiki.ros.org/roslaunch)
- [Testing: ROS USB Camera drivers](http://www.iheartrobotics.com/2010/05/testing-ros-usb-camera-drivers.html)
- [usb_cam](http://wiki.ros.org/usb_cam)
- [image_view](http://wiki.ros.org/image_view)
- [rqt](http://wiki.ros.org/rqt)

