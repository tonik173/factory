# Model and simulate a differential drive mobile robot in ROS2 Jazzy Jalisco and Gazebo starting from URDF and Xacro XML files

- robot.xacro - this is the URDF and Xacro file defining the robot geometry.
- robot.gazebo – this is the additional Gazebo file defining the plugins and additional simulation parameters. 
- bridge_parameters.yaml – this is the yaml file defining the parameters for establishing ROS2 – Gazebo Harmonic bridge. 
- gazebo_model.launch.py – this is the Python launch file that defines the nodes that need to be launched.

Author: Aleksandar Haber

In this ROS2 tutorial, we explain how to model and simulate a two-wheel differential drive mobile robot in ROS2 Jazzy Jalisco and Gazebo.

## STEP 1: Verify that you have the correct version of Linux and ROS2

Prerequisites: You have to make sure that you have the correct versions of ROS2 and Ubuntu installed on your system. Otherwise, the implementation presented in this tutorial might not work. This tutorial is based on ROS2 Jazzy Jalisco distribution and Ubuntu 24.04.

First of all, you need to have the following versions on Linux and ROS2

- Ubuntu 24.04
- ROS2 Jazzy Jalisco

To verify the Linux distribution, open a terminal and type:

```sh
cat /etc/os-release
```

You should see this:

```sh
PRETTY_NAME="Ubuntu 24.04 LTS"
NAME="Ubuntu"
VERSION_ID="24.04"
VERSION="24.04 LTS (Noble Numbat)"
VERSION_CODENAME=noble
ID=ubuntu
ID_LIKE=debian
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
UBUNTU_CODENAME=noble
LOGO=ubuntu-logo
```

It is important to see the version 24.04.

Let us verify the ROS2 distribution. Type

```sh
source /opt/ros/jazzy/setup.bash
printenv ROS_DISTRO
```

The output should be “jazzy”. If you see such an output, this means that ROS2 Jazzy Jalisco is installed on the computer.

## STEP 2: Install the necessary packages

Close the previous terminal and open a new terminal. Every time you want to do something in ROS2, you need to source the base environment. We do that by typing:

```sh
source /opt/ros/jazzy/setup.bash
```

Then, we need to install the necessary ROS2 and Gazebo packages. Some of these packages you might have, however, it is still an excellent idea to try to install them. If you already have them, the installation attempt will not change anything in your system.

```sh
sudo apt-get update
sudo apt-get upgrade
```

To install Gazebo Harmonic (recommended version for ROS2 Jazzy):

```sh
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

Then, you have to source the environment again in order to be able to run Gazebo. That is, you need to refresh the packages.

```sh
source /opt/ros/jazzy/setup.bash
```

To make sure that Gazebo is installed inside of ROS2 Jazzy, type this

```sh
which gz
```

You should get something like this as an output:

```sh
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
```

Then, let us try to run Gazebo by typing

```sh
gz sim
```

Gazebo GUI simulation window should open. Try to run one of the build-in simulations to make sure that Gazebo is working.

Then, let us make sure that the following packages are installed

```sh
sudo apt-get install gedit
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt-get install ros-jazzy-teleop-twist-keyboard
```

Let print the list of all Gazebo related packages:

```sh
ros2 pkg list | grep gz
```

It is important to see ros_gz and ros_gz_bridge. These two packages are very important for displaying the model in Gazebo and for using ROS2 Jazzy to control a model in Gazebo.

## STEP 3: Create the workspace and package

Close the previously opened terminals, and open a new terminal, and source the environment:

```sh
source /opt/ros/jazzy/setup.bash
```

Then, create the workspace folders

```sh
cd ~
mkdir -p ~/ws_mobile/src
```

Here, ws is an abbreviation for Workspace, and “mobile” is the name. Here, we are actually creating two folders. The first folder is “ws_mobile” and inside of this folder, we created another folder called “src”. Next, we create and build the workspace:

```sh
cd ~/ws_mobile/
colcon build
```

Next, we need to create the package:

```sh
cd ~/ws_mobile/src
ros2 pkg create --build-type ament_cmake mobile_robot
```

The name of the package is “mobile_robot”. The package source files and folders will be in the folder `~/ws_mobile/src/mobile_robot/ `. Consequently, let us navigate to that folder and let us create several subfolders:

```sh
cd ~/ws_mobile/src/mobile_robot/
mkdir launch model parameters
```

Here, we created three folders: “launch”, “model”, and “parameters”.

The “launch” folder will contain the Python launch file necessary to run the model in ROS2 and Gazebo.

The “model” folder will contain the Xacro and URDF source files of the model. Note that URDF stands for “Unified Robotics Description Format”. This is a file format that is used to specify the geometry of robots and some other properties. Xacro is an XML macro language used to parametrize and simplify URDF model development.

The “parameters” folder will be used to create a yaml file that will define the parameters necessary to bridge ROS2 and Gazebo topics. That is, it will define parameters that are necessary to redirect ROS2 to Gazebo topics and vice-versa. This is a new requirement introduced in Gazebo Harmonic.

To build the workspace and packages, you need to navigate to the source folder of the workspace:

```sh
cd ~/ws_mobile/
colcon build
```

## Step 4: Create the URDF (Xacro), Gazebo, and Parameter Files

First, we will create the robot model. We create the robot model by creating two files: Xacro and Gazebo files. The first Xacro file contains the robot 3D model, and the second Gazebo file contains additional parameters and Gazebo controller plugins necessary to successfully represent and control the model in Gazebo. First, let us navigate to the model folder

```sh
cd ~/ws_mobile/src/mobile_robot/model
```

Next, we need to create the Xacro model file defining the geometry. The name of the file should be “robot.xacro”. The quick approach would be to copy the included “robot.xacro” file to this folder (the file is included with this lesson). However, try to avoid this and instead type the file by yourself. Only like that you will be able to correctly learn ROS2. To create this file, either use gedit

`gedit robot.xacro`

or use the VS code editor (preferable option). After creating the file, enter the content of the file.

Then, create an additional Gazebo file that should be included in the above Xacro file. The file is provided with this tutoril. The name of the file should be “robot.gazebo”. Again, you can either use VS Code to create this file

`gedit robot.gazebo`

and enter the content of the file (file is included). Next, we need to create a yaml parameter file that will bridge ROS2 Jazzy and Gazebo. To do that, first navigate to the “parameters” folder

`cd ~/ws_mobile/src/mobile_robot/parameters`

In this folder, create the file called “bridge_parameters.yaml”. You can use VS Code or gedit to create this file. With gedit, type

`gedit bridge_parameters.yaml`

and enter the content of the file.

## STEP 5: Create the Python launch file

We create the launch file in the “launch” folder. Let us navigate to the folder

`cd ~/ws_mobile/src/mobile_robot/launch`

Create and edit the file either by using VS Code (preferable option) or by using gedit

`gedit gazebo_model.launch.py`

## STEP 6: Final adjustments and run the model

Next, we need to add the dependencies

```sh
cd ~/ws_mobile/src/mobile_robot
gedit package.xml
```

Add these lines to “package.xml”

```sh
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
```

Then, we need to adjust the CMakeLists.txt file. We need to tell to ROS2 that important files are in the two new folders “launch” and “model”

```sh
cd ~/ws_mobile/src/mobile_robot
gedit CMakeLists.txt
```

Then, above “if(BUILD_TESTING)”, type the following

```sh
install(
 DIRECTORY launch model parameters
 DESTINATION share/${PROJECT_NAME}
)
```

Next, we need to perform one final build of the workspace and the package

```sh
cd ~/ws_mobile
colcon build
```

Then, we need to source the package. That is we need to create an overlay

```sh
source /opt/ros/jazzy/setup.bash
source ~/ws_mobile/install/setup.bash
```

Finally, run the model in Gazebo:

`ros2 launch mobile_robot gazebo_model.launch.py`

The main Gazebo window with the model will start.

## STEP 7: Control the robot by using keyboard keys

Do not close the original terminal that is running Gazebo. Open a new terminal, and in the new terminal type:

```sh
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This will source the environment and start the teleop_twist_keyboard node that is used to control the robot.

## STEP 8: Investigate topics, messages, etc.

Here, we will learn how to obtain more information about the topics connecting the nodes and messages that are being sent through nodes.

To see the list of topics, open a new terminal and type:

```sh
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

The output should look like this:

```sh
/clock
/cmd_vel
/joint_states
/odom
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

To see the messages being sent through the /odom (odometry) topic, you should type:

`ros2 topic echo /odom`

To obtain information about the type of messages that are being sent through a particular topic, type

`ros2 topic info /cmd_vel`