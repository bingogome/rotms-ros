# ROTMS 

ROTMS is a modular ROS1-based system for medical image-guided robotic interventions. It enables communication between various system components, including medical imaging, robot control, target visualization, and registration. See also the navigation module of this project [rotms-slicer](https://github.com/bingogome/rotms-slicer/)

As an example, it is used for Transcranial Magnetic Stimulation.

![architecture_tms](https://github.com/bingogome/rotms-ros/blob/main/architecture.png)

## System Architecture

The system is organized into several ROS packages:

- **rotms_ros_comm**: Handles communication between ROS nodes and external modules using UDP
- **rotms_ros_comm_decode**: Decodes incoming messages based on a command dictionary
- **rotms_ros_comm_relay**: Relays messages between different components
- **rotms_ros_dispatcher**: Coordinates operations between different state machines
- **rotms_ros_kinematics**: Manages transformations between coordinate frames
- **rotms_ros_messages**: Contains custom message and service definitions
- **rotms_ros_misc**: Contains miscellaneous utility functions and nodes
- **rotms_ros_operations**: Implements operational procedures
- **rotms_ros_robot_interface**: Interfaces with the robot controller
- **rotms_ros_state**: Implements state machines for different system modules
- **rotms_ros_utility**: Contains utility functions and nodes

## Installation

### Prerequisites

- Ubuntu 18.04 or 20.04
- ROS Melodic or Noetic
- YAML-CPP library
- Boost libraries
- Eigen3

### Installation Steps

1. **Install ROS**

   Follow the [official ROS installation guide](http://wiki.ros.org/ROS/Installation) to install ROS Melodic (Ubuntu 18.04) or Noetic (Ubuntu 20.04).

2. **Install Dependencies**

   ```bash
   sudo apt-get update
   sudo apt-get install python-yaml cmake libboost-all-dev libeigen3-dev
   sudo apt-get install ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-ros
   sudo apt-get install python-trimesh python-yaml
   sudo apt-get install libyaml-cpp-dev
   ```

3. **Create a Catkin Workspace**

   ```bash
   mkdir -p ~/rotms_ws/src
   cd ~/rotms_ws/src
   ```

4. **Clone the Repository**

   ```bash
   git clone https://github.com/bingogome/rotms_ros.git
   ```

5. **Build the Workspace**

   ```bash
   cd ~/rotms_ws
   catkin_make
   ```

6. **Source the Workspace**

   ```bash
   source ~/rotms_ws/devel/setup.bash
   ```

   Add this line to your `~/.bashrc` file to source the workspace automatically:

   ```bash
   echo "source ~/rotms_ws/devel/setup.bash" >> ~/.bashrc
   ```

## System Configuration

The system components are configured through YAML files located in their respective packages:

- **Robot Configuration**: `rotms_ros_robot_interface/config.yaml`
- **Communication Configuration**: `rotms_ros_comm/config_comm.yaml`
- **Command Decoding**: `rotms_ros_comm_decode/config_comm_decode.yaml`
- **ICP Registration**: `rotms_ros_utility/icp_config.yaml`

### Pose Calibration Configuration

The system relies on several calibrated transformations that are stored in YAML files in the `rotms_ros_kinematics/share/` directory:

1. **Tool to Tool Reference (`tool_toolref.yaml`)**:
   - Transformation from the tool frame to the tool reference frame (tracked marker on the tool)
   - Used for tool tracking and visualization

2. **Tool Reference to End Effector (`toolref_eff.yaml`)**:
   - Transformation from the tool reference frame to the robot end-effector frame
   - This is the hand-eye calibration result

3. **Pointer to Pointer Tip (`ptr_ptrtip.yaml`)**:
   - Transformation from the pointer frame to the pointer tip
   - Used for digitization

4. **Contact Offset (`cntct_offset.yaml`)**:
   - Defines the offset from the contact point to the planning target
   - Default is 70mm in the z-direction

5. **Tool Offset (`offset_tool.yaml`)**:
   - Defines additional offset between the tool and contact point

These calibration files can be modified for different tools and configurations. The system includes backup calibration files for different tools (e.g., coil, femur) in the `rotms_ros_kinematics/share/backups/` directory.

Example calibration format:
```yaml
# Example tool_toolref.yaml
x: 0.002031963517013
y: 0.032358090207432
z: 0.083623735776365
rx: -0.248360560836984
ry: -0.312551065559429
rz: -0.858829880812969
rw: 0.320998596665446
```


## Running the System

1. **Launch the Complete System**

   ```bash
   roslaunch rotms_ros all.launch
   ```

2. **Launch Individual Components**

   ```bash
   # Communication nodes
   roslaunch rotms_ros_comm comm_nodes.launch
   
   # Decoding nodes
   roslaunch rotms_ros_comm_decode comm_decode_nodes.launch
   
   # Robot interface
   roslaunch rotms_ros_robot_interface interface_nodes.launch
   
   # Kinematics
   roslaunch rotms_ros_kinematics kinematics_nodes.launch
   
   # Utility nodes
   roslaunch rotms_ros_utility utility_nodes.launch
   ```

## System Operation

The system operates using a state machine architecture that manages different states for:

- **Registration**: Landmark planning, digitization, and registration
- **Digitization**: Automatic and individual landmark digitization
- **Tool Planning**: Planning and execution of tool poses
- **Robot Control**: Connection and motion control of the robot

Messages are communicated between components using standard ROS topics and services, with a dispatcher coordinating operations between different state machines.

## Key Features

- **Registration**: Point-based registration with ICP refinement
- **Robot Control**: Interface for KUKA LBR iiwa robots
- **UDP Communication**: For interfacing with external applications (e.g., medical imaging software)
- **Real-time Visualization**: For target visualization and tracking
- **State Machine Architecture**: For robust system operation

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

@article{liu2025image,
  title={An Image-Guided Robotic System for Transcranial Magnetic Stimulation: System Development and Experimental Evaluation},
  author={Liu, Yihao and Zhang, Jiaming and Ai, Letian and Tian, Jing and Sefati, Shahriar and Liu, Huan and Martin-Gomez, Alejandro and Kheradmand, Amir and Armand, Mehran},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  publisher={IEEE}
}
