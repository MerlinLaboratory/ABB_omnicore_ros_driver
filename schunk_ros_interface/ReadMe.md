# ROS drivers for EGK Schunk Grippers

**NOTE:** the support for this branch has been **dropped**. I will answer issues and problems but I will not add any additional features than the already present ones.

This repository serves as a ROS wrapper for controlling EGK Schunk grippers with Ethernet/IP connection. The repo leverages the [EIPScanner](https://github.com/nimbuscontrols/EIPScanner) repository to establish the necessary Ethernet/IP connection with the gripper. 

## Hardware Dependencies

**OS**: Linux Ubuntu \
**Gripper**: EGK-40-EI-M-B

Tested on Ubuntu 20.04 with ROS Noetic. \
While the code has been **exclusivelly tested** with the **EGK40** model, support for all other EGK models is not **excluded**.

## Features

- **Gripper State Logging**: Access real-time information about the gripper's state through a dedicated ROS topic.
- **Control Services**: Utilize ROS services to command various gripper actions, such as:
  - `jog_to`: Move the gripper to a specified position.
  - `release`: Release any grasped object.
  - `simple_grip`: Perform a simple gripping action.

Additional information about each service can be found within the `schunk_interface` package.

## Repository Structure

The principal packages are briefly described in the following table:

| Package | Description |
| --- | --- |
| [Doc](Doc) | (Contains the necessary documentation to understand how to use the gripper and the .EDS file for Ethernet/IP Assembly communication. All the files can be found on the Schunk website.). |
| [schunk_hardware_interface](schunk_hardware_interface) |  Contains the actual code responsible for establishing the connection with the gripper and providing the necessary services and topics.  |
| [schunk_interfaces](schunk_interfaces) | Contains information about the messages and services used by this repository. |

## Hardware Connection

Taking as  reference the gifs in the next paragraphs, the gripper exposes two cables: 
  - the green one for communication with external devices
  - the black one that powers the system

To connect the green cable with the ethernet port of your computer you will need to buy an [adapter](https://us.rs-online.com/product/rs-pro/2017169/72296280/). The black cables that powers the system is composed of 4 cables. Two provide power to the gripper and two provide power to the control system. These two need to be supplied with a tension of 24V and can be connected as the left for testing purposes. On the right instead how they are connected to the robot. Notice that the brown and black connectors are the source (+24V) while the white and the blue are the ground (0V).

| Testing | Robot connection |
|:---:|:---:|
| ![Testing connection](Doc/img/TestingConnection.jpg) | ![Robot connection](Doc/img/RobotConnection.jpg) |

## Usage

### Prerequisites and Dependencies
ROS2 Humble needs to be installed and configured in your computer.

The only other dependency that the repo requires is [EIPScanner](https://github.com/nimbuscontrols/EIPScanner). Just clone it and build it (it is not necessary to put this repo in your ROS workspace):
  ```
  git clone https://github.com/nimbuscontrols/EIPScanner.git
  mkdir build && cd build
  cmake ..
  cmake --build . --target install
  ```

  Finally, in the same terminal where you are going to launch the gripper node, run:
  ```
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
  ```
  otherwise place it inside your .bashrc file to avoid doing this step everytime you open a new terminal where you launch the schunk_node.

### Installation
1. Clone the repository into your ROS2 workspace and build it:
    ```
    git clone https://github.com/MerlinLaboratory/schunk_ros_interface.git --branch ros_1
    catkin build
    ```

### Usage + examples
1. **Launch the Node**: Set the gripper ip in [config](schunk_hardware_interface/config/params.yaml) file and launch the node to start communication with the gripper.
    ```
    roslaunch schunk_hardware_interface gripper.launch
    ```
    See [here](#troubleshooting) for problems/errors.
2. **Access Gripper State**: Subscribe to the gripper state topic to receive updates on the gripper's status.
    ```
    rostopic echo /schunk/egk_40_state
    ```

3. **Control the Gripper**: Call the provided services to command the gripper's actions. For example (see each srv for the params meaning):
    ```
    rosservice call /schunk/egk_40/jog_to "position: 0.0 velocity: 6.0 motion_type: 0"
    ```
    <img src="Doc/img/JogTo.gif" width="400" height="600" />
    
    ```
    rosservice call /schunk/egk_40/simple_grip "gripping_force: 50 gripping_direction: 0"
    ```
    <img src="Doc/img/SimpleGrip.gif" width="400" height="600" />

    ```
    rosservice call /schunk/egk_40/release "{}"
    ```
    <img src="Doc/img/Release.gif" width="400" height="600" />

# Troubleshooting
### No Data received from gripper / gripper does not respond to commands
In case the gripper does not communicates with your system, try to allow UDP connection through port 2222 with the following command (pay attenction that this needs to be done everytime the computer is restarted):
```
sudo iptables -A OUTPUT -p udp -m udp --sport 2222 -j ACCEPT
sudo iptables -I INPUT -p udp --dport 2222 -j ACCEPT
```

### Node crashes at launch with communication error
If the node of the gripper crashes at launch, be sure that your computer running ROS has exclusive ownership of the device. For example, in our setup, we have all the devices connected to a switch, creating a subnetwork under 192.168.125.X, as the following figure shows: 

<img src="Doc/img/Setup.png" width="800" height="600" />

As soon the gripper is connected to the subnetwork, the robot might take exclusive ownership of the gripper, making the code crash. 
Unluckily, I cannot provide a unique solution since it is robot-dependent. However, you need to make the robot 'forget' the gripper. In my case, we have a GoFa from ABB with the Schunk App installed on the Teach Pendant; we just had to click 'Delete Gripper' once I opened the App.

## Future Development

None