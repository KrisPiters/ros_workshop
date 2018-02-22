# Workshop "ROS for Engineers"

Author (NL): Eric Dortmans (e.dortmans@fontys.nl)

Translation: Kris Piters (k.piters@fontys.nl)

## Learning goals

1. Know and understand basic concepts of ROS
2. Being able to deal with various ROS tools in a practical way
3. Being able to build an application yourself with existing ROS nodes
4. Being able to Model, visualize and simulate robots
5. Being able to make a mobile robot platform navigate autonomously
6. Being able to plan a trajectory for a robot arm

To achieve these learning objectives, a number of sessions are required.

## Foreknowledge

Required prior knowledge:

- Some skills in dealing with Linux (especially Ubuntu), both with the mouse and via commands in a terminal window
- Some knowledge of sensors (camera, laser scanner, etc.), and actuators (motors etc.)
- Some knowledge of kinematics
- Some knowledge of interfaces (USB, RS232, Ethernet / EtherCat)

Programming experience is not necessary, but useful.

## Necessities

You have to bring your own laptop with installed Ubuntu 16.04 and ROS Kinetic. You can install this software in 2 ways:

1. as a virtual machine under Windows (or Mac OS / X)
2. native, for example as a multiboot option besides Windows

### Ubuntu and ROS in a Virtual Machine

Running ROS in a Virtual Machine has the advantage that you do not have to install Ubuntu and ROS yourself. The disadvantage is that the performance is less than a native installation. A native installation is recommended for running 3D simulations.

All you have to do is download 2 files:

- an Ubuntu plus ROS image
- a program such as VirtualBox to execute that image

On the [Nootrix site] (http://nootrix.com/) there are a number of [ready-to-use ROS Indigo images] (http://nootrix.com/2014/09/ros-indigo-virtual-machine/). The 32 bit image (RosIndigo32Bits.ova) is good enough. You can download it via your web browser [here] (http://www.fhict.nl/docent/downloads/TI/MinorES/RosIndigo32Bits.ova). You can also download it [via Bittorrent] (http://nootrix.com/00download/download.html?fileId=rosIndigo32BitsVMTorrent).

You can download the program VirtualBox for Windows (or Mac OS / X) [here] (https://www.virtualbox.org/wiki/Downloads). Then you have to install it just like any other program.
Instead of VirtualBox you can also use VMware. You can download the VMware Player [here] (http://www.filehippo.com/download_vmware_player/).

With VirtualBox (or VMware Player) you can now open the RosIndigo32Bits.ova file and start the relevant virtual machine.

### Ubuntu plus ROS native

For a native installation you have to do two things:

1. Install Ubuntu 16.04
2. Install ROS Kinetic in Ubuntu

Ubuntu Desktop 16.04 you can [download here] (http://www.ubuntu.com/download).

How to install ROS Kinetic (Desktop - full install) you can read [here] (http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Linux command skill

In order to work with ROS, you have to have some skills in dealing with Linux commands.
Some tutorials that might help:

- UNIX Tutorial for Beginners: http://info.ee.surrey.ac.uk/Teaching/Unix/
- Linux Command Line Cheat Sheet: https://www.cheatography.com/davechild/cheat-sheets/linux-command-line/

