# Workshop "ROS for Engineers" - part 2

Original author (Dutch): Eric Dortmans (e.dortmans@fontys.nl)

Translation / Update (English): Kris Piters (k.piters@fontys.nl)

## Preperation

Update the ros_examples package you installed in the previous exercise:

    cd ~/catkin_ws/src/ros_examples
    git pull
    cd ~/catkin_ws
    catkin_make

Install the packages needed for simulating the Turtlebot:

    sudo apt-get install ros-kinetic-turtlebot
    sudo apt-get install ros-kinetic-turtlebot-apps
    sudo apt-get install ros-kinetic-turtlebot-rviz-launchers
    sudo apt-get install ros-kinetic-turtlebot-simulator

Install extra packages for use with [Stage](http://wiki.ros.org/stage#External_Documentation):

    sudo apt-get install ros-kinetic-mouse-teleop
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    sudo apt-get install ros-kinetic-stage
    sudo apt-get install ros-kinetic-stage-ros

## Controlling a simulated robot in Stage

In this chapter we'll use Stage, a 2D robot simulator. 

Start a simulated robot in a simulated world:

    roslaunch stage_worlds kinect_world.launch

Use another terminal window to start the *teleop_twist_keyboard* node for controlling the robot:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Use another window to analyze the odometry information published by the robot:

    rostopic echo /odom

Use the following command to see the structure of a *Odometry* message:

    rostopic type /odom | rosmsg show
  
Watch how the odometry information will change by driving around.

Start [*rviz*](http://wiki.ros.org/rviz) for extra visualization of data produced by the robot:

    roslaunch stage_worlds rviz_stage.launch

## [TF](http://wiki.ros.org/tf) 

Take a look at the *tf tree* while the simulated world is active:

    rosrun rqt_tf_tree rqt_tf_tree

Alternatively use rqt and its tf tree plugin:

    rqt

    Plugins > Visualization > TF Tree

Take a look at what's being published to the tf tree with [tf_monitor](http://wiki.ros.org/tf#tf_monitor):

    rosrun tf tf_monitor

Request specific, single transformations with [tf_echo](http://wiki.ros.org/tf#tf_echo):
       
    rosrun tf tf_echo /base_link /base_laser_link
    rosrun tf tf_echo /base_footprint /base_link
    rosrun tf tf_echo /odom /base_footprint
    
You can also request complex, multiple transformations, e.g.:

    rosrun tf tf_echo /odom /base_laser_link

## Navigation

Stop your running nodes (Stage, Rviz, etc). To be sure you could close all terminal windows.

We are now going to experiment with the ROS Navigation stack.

Start *gmapping* to make a map of the (simulated) robot world:

    roslaunch stage_navigation kinect_gmapping.launch

Start the *teleop_twist_keyboard* node to the robot:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    
Alternativly Start de *arbotix gui* om met je muis de robot te besturen:

    arbotix_gui

Je kunt overigens de robot in Stage ook verplaatsen door hem met je muis te selecteren en dan te "verslepen".

Rij de robot rond door de gesimuleerde wereld. Kom ook regelmatig terug waar je al geweest bent.
In het RViz window kun je dan zien hoe gmapping een kaart (map) van de wereld probeert te maken.

Je zult merken dat het niet meevalt om een goede map te maken! 

We zullen een andere robot proberen. Stop de lopende simulatie en start de volgende:

    roslaunch stage_navigation laser_gmapping.launch
    
Zie en merk je verschil? Wat neem je waar en hoe zou dat komen?

Als je tevreden bent met je map kun je hem opslaan:

    rosrun map_server map_saver -f /tmp/world_map

Stop de lopende lsimulatie. Voor de zekerheid sluit alle terminal windows.

Nu we een kaart hebben kunnen we die door de robot laten gebruiken om voortaan zelf zijn weg te zoeken.
We gebruiken daarvoor de *amcl* localizatie node en de *move_base* navigatie node:

    roslaunch stage_navigation kinect_amcl.launch map_file:=/tmp/world_map.yaml

Bekijk de Computation Graph:

    rqt_graph

Gebruik the *2D Nav Goal* knop in RViz om robot een doel (goal) te geven. Click eerst op de knop en dan op een plaats in de map. Stuur hem bijvoorbeeld naar linksboven en vervolgens naar rechtsonder in de map.

Stop de draaiende Stage simulatie. Voor de zekergeheid sluit alle terminal windows.

## Besturen van een gesimuleerde Turtlebot in Stage

We zullen nu hetzelfde doen maar met een robot die ook echt bestaat: de Turtlebot.

Start de Turtlebot sofware:

    export TURTLEBOT_BASE=roomba
    export TURTLEBOT_STACKS=circles
    export TURTLEBOT_3D_SENSOR=kinect
    export TURTLEBOT_STAGE_MAP_FILE=`rospack find turtlebot_stage`/maps/maze.yaml
    export TURTLEBOT_STAGE_WORLD_FILE=`rospack find turtlebot_stage`/maps/stage/maze.world
    roslaunch turtlebot_stage turtlebot_in_stage.launch

Bekijk de Computation Graph:

    rqt_graph

Wat zou de cmd_vel_mux doen? Lees de parameter file (een YAML file) van de cmd_vel_mux:

    cat `rospack find turtlebot_bringup`/param/mux.yaml

Gebruik het keyboard om de robot te besturen:

    roslaunch turtlebot_teleop keyboard_teleop.launch

De robot kan ook autonoom navigeren. Gebruik de *2D Nav Goal* knop in RViz om robot een doel (goal) te geven.

Merk op dat de Turtlebot mooi gevisualiseerd wordt in RViz. Hoe dat moet gaan we volgende keer bekijken.

## EXTRA: Rijden van een gesimuleerde Turtlebot in Gazebo

Gazebo is een 3D simulator. Dit vraagt natuurlijk meer van de grafische ondersteuning en van de performance van je laptop, maar is wel veel echter.

Start de Gazebo simulator:

    export TURTLEBOT_BASE=roomba
    export TURTLEBOT_STACKS=circles
    export TURTLEBOT_3D_SENSOR=kinect
    export TURTLEBOT_GAZEBO_WORLD_FILE=`rospack find turtlebot_gazebo`/worlds/playground.world
    roslaunch turtlebot_gazebo turtlebot_world.launch
  
Run de *gmapping* node:

    roslaunch turtlebot_gazebo gmapping_demo.launch

Visualizeer de robot tijdens navigatie:
    
    roslaunch turtlebot_rviz_launchers view_navigation.launch

Start de *arbotix_gui* om de robot te besturen:

    arbotix_gui cmd_vel:=cmd_vel_mux/input/teleop

De simulatie bootst ook een echte kinect na. Voeg maar eens een Camera toe in RViz op het topic /camera/rgb/image_raw

## EXTRA: Navigeren met een echte Turtlebot

Voor het installen van *ROS_IP* en *ROS_MASTER_URI* zie laatste opdracht van vorige sessie.

Start de robot, inclusief gmapping node.

Bekijk en bestuur de robot via RViz op je laptop:

    roslaunch turtlebot_rviz_launchers view_navigation.launch

Voeg ook een Camera toe op het /camera/rgb/image_raw topic.

## Referenties
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [TF](http://wiki.ros.org/tf)
- [Introduction to TF](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf)
- [rqt](http://wiki.ros.org/rqt)
- [navigation](http://wiki.ros.org/navigation)
- [TurtleBot](http://wiki.ros.org/Robots/TurtleBot)
- [turtlebot_simulator](http://wiki.ros.org/turtlebot_simulator)
- [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation)

