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

- What type of message is associated with the topic */turtle1/cmd_vel*?

- What happens when using the following command?

```rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}}"```
    
And what about this command?

    rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{angular: {z: 1.57}}"

Draw a square with the turtle (approximately).

- What happens when you execute the following command?

```rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"```

How does it compare to this command:
    
```rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'```
      
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
    
To stop recording hit *CTRL-C* in the terminal running rosbag.

Reset *turtlesim* to start with a clean slate:

    rosservice call /reset
    
Play back the recorded messages stored in the *bag file*:

    rosbag play /tmp/turtle1.bag

- What happens?

## launch files

Tot nu toe hebben we steeds alle nodes (plus natuurlijk roscore) stuk voor stuk vanuit een nieuw terminal window moeten opstarten. Dat is natuurlijk niet handig. Gelukkig heeft ROS een mechanisme om een verzameling nodes in een keer te lanceren met behulp van het commando *roslaunch*. Welke nodes met welke parameters moeten worden opstart moeten we natuurlijk wel eerst specificeren. Zo'n specificatie heet een *launch file*.

Bestudeer de launchfiles van het *agitr* package. Met het volgende commando open je betreffende launch directory met de Ubuntu filemanager (*nautilus*):

    nautilus `rospack find agitr`/launch
    
Probeer ze uit met het *roslaunch* programma:

    roslaunch agitr *launchfile_naam*

We gaan nu zelf een nuttige launch file maken voor het opstarten van benodigde nodes om realtime beeld van een webcam op ons scherm te laten zien. Tegenwoordig hebben veel laptops zo'n webcam boven aan het scherm zitten voor b.v. Skype.

Installeer een USB camera driver package:

    sudo apt-get -y install ros-indigo-usb-cam 

Dit package bevat een node voor de aansturing van de USB webcam.

We hebben ook een node nodig die images kan lezen en op het scherm kan ztten. We gebruiken daarvoor de *image_view* node van het *image_view* package. Dit package is al standaard geinstalleerd.

We willen nu beide nodes tegelijk opstarten met de benodigde parameters. Daarvoor moeten we een launch file maken. We zullen eerst een nieuw package creeren om die launchfile in te zetten. Laten we het *usb_camera* noemen.

    cd ~/catkin_ws/src
    catkin_create_pkg usb_camera std_msgs rospy roscpp
    cd ~/catkin_ws
    catkin_make

Even checken of ROS ons package nu kan vinden:

    rospack find usb_camera

In ons nog vrij lege package maken we een directory *launch*. Daar gaan we onze launch file in zetten.

    roscd usb_camera
    mkdir launch
    
Waarschijnlijk handiger om nu deze directory met de Ubuntu filemanager te openen:

    nautilus `rospack find usb_camera`/launch
    
Maak nu in deze launch directory een nieuwe file aan met de naam *usb_camera.launch* en geef deze file de volgende inhoud:

    <launch>
      <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
        <param name="video_device" value="/dev/video0" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <param name="pixel_format" value="mjpeg" />
        <!-- <param name="pixel_format" value="yuyv" /> -->
        <param name="camera_frame_id" value="usb_cam" />
        <param name="io_method" value="mmap"/>
      </node>
      <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
        <!-- <remap from="image" to="/usb_cam/image_raw"/> -->
        <param name="autosize" value="true" />
      </node>
    </launch>

Let op: meestal is het webcam device in Ubuntu geregistreerd als */dev/video0*, maar het kan ook bijvoorbeeld */dev/video1* zijn.

Let op: niet alle webcams zijn hetzelfde. Soms moet je als pixel_format *mjpeg* gebruiken en soms *yuyv*. Even uitproberen dus.

Test de launchfile:

    roslaunch usb_camera usb_camera.launch

Bekijk de computation graph

    rqt_graph

Welk structureel probleem zie je? Los het op door de launch file aan te passen.
Als het je lukt kun je mooi naar jezelf kijken ;-)

## rqt

Veel zaken die je kunt doen via losse commando's kun je ook doen via het programma *rqt*, de grafische user interface (GUI) van ROS. Dit programma kan ook de robot en verschillende messages visualiseren.

Start de *rqt* GUI:

    rqt

Verken de verschillende plugins.

Enkele plugins (b.v. rqt_graph, rqt_image_view) ken je al want die hebben we los opgestart en gebruikt. Binnen rqt zijn alle plugins mooi geintegreerd.

## Besturen van een echte mobiele robot

Tot nu toe hebben we alle nodes op onze eigen laptop gedraaid. Meestal draait een gedeelte van de ROS nodes echter op de robot zelf. De nodes op onze laptop moeten dan via het netwerk (b.v. Wifi) communiceren met de nodes die op de robot draaien.

ROS is gemaakt om gedistribueerd te draaien. Wel moet je weten waar de ROS Master node draait. Meestal draait die op de robot. Het is voldoende om de omgevingsvariabelen ROS_IP en ROS_MASTER_URI aan te passen. In ROS_IP moet het IP_ adres van je laptop komen. Dit kunnen we opvragen via het commando `hostname -I`. Let op: je laptop kan meerder IP adressen hebben (want elke actieve netwerk interface krijgt een eigen IP adres). Je kiest dan die van het netwerk waarmee je met de robot bent verbonden. Heb je maar een IP adres dan is het simpel:

    export ROS_IP=`hostname -I`

Anders moet je zelf het goede IPadres invullen:

    export ROS_IP=*IP_adres_van_je_laptop*

Nu moet je nog instellen hoe jouw laptop nodes de ROS Master op de robot kunnen bereiken:

    export ROS_MASTER_URI=http://*IP_adres_van_de_robot*:11311

Deze exports kun je ook in je *.bashrc* zetten, dan worden ze automatisch uitgevoerd als je een nieuw terminal window opent:

    gedit ~/.bashrc

Kijk of je met de ROS Master (die op de robot draait) kunt communiceren:

    rostopic list

Het besturen van de robot gaat nu exact hetzelfde als bij de gesimuleerde robot in turtlesim.

Laat de robot door middel van een *rostopic pub* commando een rondje draaien.

Start het volgende handige programma om de robot met je muis te besturen:

    arbotix_gui
    
Dit programma leest je muis acties (bewegen van de rode stip) en vertaalt ze in Twist messages op het cmd_vel topic.

De robot heeft ook een camera, zodat we zien waar we rijden. Maak het camerabeeld zichtbaar view de *Image View* plugin van rqt.

Open ook eens de *RViz* plugin van rqt en voeg een RobotModel display toe (gebruik Add). Je ziet dan de robot model gevisualiseerd. Daar komen we o.a. in volgende sessies op terug.

## Referenties
- [A Gentle Introduction to ROS](http://www.cse.sc.edu/~jokane/agitr/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [YAML on the ROS command line](http://wiki.ros.org/ROS/YAMLCommandLine)
- [rosparam](http://wiki.ros.org/rosparam)
- [roslaunch](http://wiki.ros.org/roslaunch)
- [Testing: ROS USB Camera drivers](http://www.iheartrobotics.com/2010/05/testing-ros-usb-camera-drivers.html)
- [usb_cam](http://wiki.ros.org/usb_cam)
- [image_view](http://wiki.ros.org/image_view)
- [rqt](http://wiki.ros.org/rqt)

