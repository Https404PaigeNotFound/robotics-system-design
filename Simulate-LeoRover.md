# **Leo Rover Simulation Setup (ROS 2 Humble)**

## **Overview**
This guide walks you through setting up a **Leo Rover simulation environment** using **ROS 2 Humble**, **Gazebo**, and **RViz** on **Ubuntu 22.04**. By the end of this tutorial, you will have a functional simulation of the Leo Rover to test and develop autonomous robotics applications.

Having a simulation environment is particularly useful because, when using the Leo Rover physically, ROS 2 nodes are distributed between the **Raspberry Pi** onboard the rover and an **Intel NUC** for additional processing. A simulation environment allows for development and testing of code without requiring physical access to the robot, making it easier to iterate on algorithms, debug issues, and refine autonomy features before deploying them to the real hardware.

---

## **Prerequisites**
Ensure the following are installed on your system before proceeding:

- **Ubuntu 22.04 LTS (native install or through WSL)**
- **ROS 2 Humble**
  - [Source Installation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
  - [Debian Package Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **Colcon (Build System)**
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
- **Gazebo (Ignition)**
  ```bash
  sudo apt install ros-humble-ros-ign
  ```
- **Additional Dependencies**
  ```bash
  sudo apt install python3-rosdep python3-vcstool python3-colcon-common-extensions
  ```

---

## **Setting Up a GitHub Repository**
To promote good practice, use GitHub for version control. You will need to create a repository if you don't already have one.
### **Step 1: Clone your GitHub Repository**
In this guide the repository is called `leo_rover_simulation`.
```bash
cd ~
git clone https://github.com/YOUR_GITHUB_USERNAME/leo_rover_simulation.git
cd leo_rover_simulation
```

### **Create a ROS 2 Workspace**
```bash
mkdir -p ros2_ws/src
cd ros2_ws
```

---

## **Python Virtual Environment Setup**

### **Create and Activate a Virtual Environment**
```bash
cd ~/leo_rover_simulation
python3 -m venv venv
source venv/bin/activate
```

### **Upgrade `pip`**
```bash
pip install --upgrade pip
```

### **Install Python Dependencies**
Create a `requirements.txt` file with the any python packages you'd like to install:
```bash
cat <<EOL > requirements.txt
numpy
opencv-python
matplotlib
pillow
rclpy
PyYAML
tk
EOL
```
Then install them:
```bash
pip install -r requirements.txt
```

---

## **Clone Required ROS 2 Packages**
```bash
cd ~/leo_rover_simulation/ros2_ws/src
git clone -b humble https://github.com/LeoRover/leo_simulator-ros2.git
git clone https://github.com/LeoRover/leo_common-ros2.git
```

Then, install the necessary ROS dependencies:
```bash
cd ~/leo_rover_simulation/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## **Verify OpenGL Support**
```bash
glxinfo | grep "OpenGL version"
```
If `glxinfo` is not found, install:
```bash
sudo apt install mesa-utils
```

---

## **Build the ROS 2 Workspace**
```bash
cd ~/leo_rover_simulation/ros2_ws
colcon build --symlink-install
```
_If you encounter errors, try cleaning the workspace and rebuilding:_
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```
_The `--symlink-install` flag allows for faster development cycles by avoiding the need to rebuild the workspace after every change. If issues with building the workspace occur then remove the `--symlink-install` flag._

---

## **Source the Workspace**
```bash
source install/setup.bash
```
_Optional: To automatically source it in every terminal session:_
```bash
echo "source ~/leo_rover_simulation/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## **Configure Environment Variables**
To prevent issues related to conflicting Gazebo environment variables.
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/humble/share:/opt/ros/humble/share/leo_gz_worlds/worlds:/opt/ros/humble/share/leo_gz_worlds/models
```
_To make these settings permanent:_
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/humble/share:/opt/ros/humble/share/leo_gz_worlds/worlds:/opt/ros/humble/share/leo_gz_worlds/models' >> ~/.bashrc
source ~/.bashrc
```

---

## **Launch the Simulation**

### **Activate the Virtual Environment**
```bash
cd ~/leo_rover_simulation
source venv/bin/activate
```

### **Source ROS 2 and Workspace Setup Scripts**
```bash
source /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
source ~/leo_rover_simulation/ros2_ws/install/setup.bash
```

### **Start the Simulation**
```bash
ros2 launch leo_gz_bringup leo_gz.launch.py
```

---

## **🛠 Troubleshooting**

### **1. Simulation Crashes Due to OpenGL Errors**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```
_To make it permanent:_
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

### **FastRTPS Shared Memory Error**
Check if multiple instances of Gazebo, RViz or ROS2 are running, if so close them.

### **Incorrect `/run/user/1000/` Permissions**
```bash
chmod 0700 /run/user/1000
```

---

## **Commit and push your updates to Github**
Now is a good time to commit and push your updates to GitHub. From now on the guide will not prompt you to do this but you are encouraged to backup your work using GitHub.
```
git commit -m "Your commit message"
```
```
git push origin main
```

---


> [!CAUTION]
> Guidance below this point is still in development.

## **Creating a Simple GUI Controller for the Leo Rover Simulation**

A **Graphical User Interface (GUI) controller** can be useful for manually driving the Leo Rover in simulation. This section provides step-by-step instructions for creating a simple GUI using **Tkinter (Python’s standard GUI library)** to publish velocity commands to the rover.

### **Create a New ROS 2 Package**
Navigate to your ROS 2 workspace and create a new package for the GUI controller:
```bash
cd ~/leo_rover_simulation/ros2_ws/src
ros2 pkg create --build-type ament_python gui_controller_leo
```

This command creates a new package named `gui_controller_leo` with the **ament_python** build type, which is ideal for Python-based nodes.

### **Install Tkinter (if not already installed)**
```bash
sudo apt install python3-tk
```

### **Create the GUI Node**
Navigate to your package directory:
```bash
cd ~/leo_rover_simulation/ros2_ws/src/gui_controller_leo
```
Create a `gui_controller.py` file inside `gui_controller_leo/`:
```bash
touch gui_controller_leo/gui_controller.py
chmod +x gui_controller_leo/gui_controller.py
```

Edit `gui_controller.py` and add the following code:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
import threading

class GUIController(Node):
    def __init__(self):
        super().__init__('leo_rover_gui_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        
        # Start ROS spin in a background thread
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.ros_thread.start()
        
        # Start the GUI in the main thread
        self.create_gui()

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Leo Rover Controller")
        self.root.geometry("300x200")

        tk.Button(self.root, text="Forward", command=self.move_forward).pack()
        tk.Button(self.root, text="Backward", command=self.move_backward).pack()
        tk.Button(self.root, text="Left", command=self.turn_left).pack()
        tk.Button(self.root, text="Right", command=self.turn_right).pack()
        tk.Button(self.root, text="Stop", command=self.stop).pack()

        self.root.mainloop()  # Keep Tkinter running in the main thread

    def move_forward(self):
        self.twist.linear.x = 0.5
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def move_backward(self):
        self.twist.linear.x = -0.5
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 1.0
        self.publisher_.publish(self.twist)

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -1.0
        self.publisher_.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

def main():
    rclpy.init()
    gui_controller = GUIController()

    # Keep the GUI running in the main thread
    gui_controller.root.mainloop()

    # Cleanup when GUI is closed
    gui_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
Save the file.

> [!NOTE]  
> Currently this WIP is having issues with the launch files. The GUI node works therefore you can skip making the launch file and build the node, then try the following:
> ```ros2 launch leo_gz_bringup leo_gz.launch.py```,
> Ctrl + z,
> ```bg```,
> ```ros2 run gui_controller_leo gui_controller```.

### **Create a `launch/` Directory and Launch file**
Navigate to your package and create a `launch` folder if it doesn't exist:
```
cd ~/leo_rover_simulation/ros2_ws/src/gui_controller_leo
mkdir -p launch
```
Inside `launch/`, create a Python launch file, e.g., `gui_controller_launch.py`:
```
touch launch/gui_controller_launch.py
```
Make sure the file is executable:
```
chmod +x launch/gui_controller_launch.py
```
Edit `gui_controller_launch.py` and add the following code:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Get package directories
    pkg_leo_simulator = get_package_share_directory("leo_simulator")

    # Include the existing Leo Rover simulation launch file
    leo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_leo_simulator, "leo_gz_bringup", "launch", "leo_gz.launch.py"))
    )
    #leo_simulation = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(pkg_leo_gz_bringup, "launch", "leo_gz.launch.py"))
    #)

    """ 
    # Launch RViz
    rviz_config_path = os.path.join(pkg_leo_simulator, "rviz", "default.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )
    """
    # Launch the GUI controller
    gui_controller = Node(
        package="gui_controller_leo",
        executable="gui_controller",
        name="gui_controller",
        output="screen",
    )

    return LaunchDescription([
        leo_simulation,  # Starts Gazebo with the Leo Rover
        #rviz_node,       # Starts RViz for visualisation
        gui_controller,  # Starts GUI for manual control
    ])
```
Save the file.

### **Update `setup.py`**
Edit `setup.py` in `gui_controller_leo/` and modify `entry_points`:
```python
    entry_points={
        'console_scripts': [
            'gui_controller = gui_controller_leo.gui_controller:main',
        ],
    },
```
Also modify `data_files`
```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/gui_controller_leo']),
    ('share/gui_controller_leo', ['package.xml']),
    ('share/gui_controller_leo/launch', ['launch/gui_controller_launch.py']),
],
```
Save the file.

### **Build the `gui_controller_leo` Package**
```bash
cd ~/leo_rover_simulation/ros2_ws
colcon build --packages-select gui_controller_leo
source install/setup.bash
```

### **(optional) Clean workspace, ensure all dependencies are installed, and rebuild the workspace**
```bash
cd ~/leo_rover_simulation/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
cd ~/leo_rover_simulation/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

### **Update `requirements.txt` (Best Practice)**
After installing dependencies, update `requirements.txt` to reflect the latest package versions:
```bash
pip freeze > ~/leo_rover_simulation/requirements.txt
```

### **Run the Launch File**
```bash
ros2 launch gui_controller_leo gui_controller_launch.py
```

### **Best Practices for ROS 2 Python Code**
- Use **meaningful class and function names** to improve readability.
- Follow **PEP8** style guidelines for Python code formatting.
- Keep functions modular to separate **GUI logic** from **ROS 2 publishing**.
- Ensure proper exception handling to prevent crashes.
- Regularly update `requirements.txt` using `pip freeze` to track installed dependencies.




