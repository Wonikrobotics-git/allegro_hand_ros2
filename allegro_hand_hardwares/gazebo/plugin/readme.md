# allegro_hand_gazebo_plugin

The `allegro_hand_gazebo_plugin` package provides the essential plugins and configurations to simulate the Allegro Hand in **Gazebo Harmonic** using the `ros2_control` framework.

This package acts as a communication bridge between the Gazebo physics engine and the ROS 2 controller manager, enabling high-fidelity simulation of the hand's kinematics and dynamics.

---

## 1. Features

* **Gazebo Harmonic Integration**: Full support for the latest Gazebo simulation environment.
* **ros2_control Interface**: Exposes hardware interfaces (Position, Velocity, and Effort) to ensure compatibility with standard ROS 2 controllers.
* **Physics Synchronization**: Ensures stable and realistic joint behavior within the Gazebo Sim environment.

---

## 2. Dependencies

This package requires the following ROS 2 and Gazebo components:

* `rclcpp`
* `hardware_interface`
* `controller_manager`
* `gz_ros2_control`
* `pluginlib`
* **Gazebo Backend**: `gz-sim` (specifically the **Harmonic** version)

---

## 3. Gazebo Harmonic Installation & Setup

This package specifically targets **Gazebo Harmonic**. By default, ROS 2 Humble often installs Gazebo Fortress. You **must** remove Fortress and its dependencies to prevent library conflicts before installing Harmonic.

### 3.1. Remove Gazebo Fortress (if previously installed)

Execute the following commands to clean up the existing Gazebo installation:

```bash
# 1. Remove ROS wrapper packages
sudo apt remove ros-humble-ros-gz

# 2. Remove Gazebo Fortress
sudo apt remove ignition-fortress

# 3. Purge all related Ignition/Gazebo packages
sudo apt purge "ignition-*" "libignition-*"

# 4. Clean up unused dependencies (Note: Review the list before confirming)
sudo apt autoremove
```

### 3.2. Install Gazebo Harmonic and ROS 2 Integration Packages

Follow these steps to add the official OSRF repository and install Gazebo Harmonic:

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

# Add the Gazebo GPG key
sudo wget [https://packages.osrfoundation.org/gazebo.gpg](https://packages.osrfoundation.org/gazebo.gpg) -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add the Gazebo stable repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] [http://packages.osrfoundation.org/gazebo/ubuntu-stable](http://packages.osrfoundation.org/gazebo/ubuntu-stable) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update

# Install Gazebo Harmonic and the ROS 2 Humble bridge for Harmonic
sudo apt-get install ros-humble-ros-gzharmonic

# (Recommended) Set the GZ_VERSION environment variable permanently
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
source ~/.bashrc
```

Verify the installation by running a sample world:

```bash
# Ensure the environment variable is set
export GZ_VERSION=harmonic

# Run a basic shapes world
gz sim shapes.sdf
```

### 3.3. Build gz_ros2_control from Source

The binary version of `ros-humble-gz-ros2-control` is typically built for Gazebo Fortress. Since this package uses **Harmonic**, you must uninstall the binary and build `gz_ros2_control` from source within your workspace to ensure compatibility.

```bash
# Remove the binary package to avoid conflicts
sudo apt remove ros-humble-gz-ros2-control

# Clone the source code into your workspace
cd ${YOUR_WORKSPACE}/src
git clone -b humble [https://github.com/ros-controls/gz_ros2_control.git](https://github.com/ros-controls/gz_ros2_control.git)
```

---

## 4. Build Instructions

**Important**: This package only supports Gazebo Harmonic. You must ensure the `GZ_VERSION` environment variable is set to `harmonic` before initiating the build process.

```bash
# Navigate to your workspace root
cd ${YOUR_WORKSPACE}

# Set Gazebo version
export GZ_VERSION=harmonic

# Build the plugin package
colcon build --packages-select allegro_hand_gazebo_plugin

# Source the workspace
source install/setup.bash
```

---

## 5. Usage

This plugin is integrated via the URDF configuration of the Allegro Hand. It enables the `ros2_control` system to communicate with the Gazebo simulation.

### 5.1. URDF/Xacro Configuration

Include the following `<ros2_control>` and `<gazebo>` tags in your URDF (or Xacro) file. The `<parameters>` tag should point to your specific controller configuration file.

```xml
<ros2_control name="AllegroHandGazebo" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    </ros2_control>

<gazebo>
    <plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimSystem">
        <parameters>$(find allegro_hand_bringup)/config/v4/single_hand/ros2_controllers.yaml</parameters>
    </plugin>
</gazebo>
```

For detailed instructions on launching the simulation and controllers, please refer to the documentation in the **`allegro_hand_bringup`** package.