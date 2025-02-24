# Integrating an Intel NUC with Leo Rover

This guide expands on the **LeoOS and ROS 2 setup** by integrating an **Intel NUC** with the **Leo Rover's Raspberry Pi**. This enhances computational capabilities, enables efficient processing distribution, and optimises performance by running resource-intensive tasks on the NUC. To facilitate node discovery through the Data Distribution Service (DDS) in ROS 2, a connection between the NUC and the Raspberry Pi is essential. This connection can be established wirelessly by configuring the Raspberry Pi as a hotspot, allowing the NUC to connect to it. Alternatively, a wired connection using an Ethernet cable can physically link the Leo Rover and the NUC.

## 1. Prerequisites
Before proceeding, ensure the following:
- The **Intel NUC** has **ROS 2 Humble** installed.
- A **ROS 2 workspace** (`ros2_ws`) is set up on the NUC.
  ```sh
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  colcon build
  source install/setup.bash
  ```
- The **Raspberry Pi** is running **LeoOS with ROS 2** configured.

## 2. Purpose & Benefits
Integrating the Intel NUC allows for **seamless communication** between both devices using **static IPs** and **ROS 2 node distribution**. This setup provides:
- **Improved Processing Power:** Offload complex tasks (e.g., SLAM, depth sensing, and robotic arm control) to the NUC.
- **Stable Networking:** Configure a direct, reliable network between the NUC and Raspberry Pi using static IPs.
- **Optimised ROS 2 Performance:** Launch and manage ROS 2 nodes across both devices for coordinated execution.
- **Automatic Node Discovery:** ROS 2 uses **Data Distribution Service (DDS)** to enable seamless discovery of nodes across the network without additional configuration.

## 3. Best Practices for Workspace Layout
When organising custom ROS 2 nodes, it's best to structure them by function. A typical ROS 2 workspace (`ros2_ws`) should look like this:
```
ros2_ws/
 ├── src/
 │   ├── robot_control/        # Package for rover motion control
 │   ├── sensor_processing/    # LIDAR, IMU, camera handling
 │   ├── gui_interface/        # GUI-based rover control
 │   ├── navigation/           # Mapping and path planning
 ├── build/
 ├── install/
 ├── log/
```
This modular approach keeps functionality separated and makes debugging easier.

### Who Defines ROS Best Practices?
- **Open Robotics** (ROS maintainers) provide [official guidelines](https://docs.ros.org/en/rolling/) for ROS development.
- **REP (ROS Enhancement Proposals)** outline conventions and best practices.
- **Industry & Research Groups** (e.g., NASA, Amazon Robotics) contribute to ROS best practices.

## 4. System Overview
### Raspberry Pi (Leo Rover):
Handles real-time hardware control and telemetry processing:
- **Motor Controller Node:** Manages wheel motor operations.
- **Battery Monitor Node:** Monitors voltage, current, and charge levels.
- **IMU Node:** Captures acceleration, gyroscope, and magnetometer data.
- **Wheel Encoder Node:** Provides odometry for movement tracking.
- **Camera Node:** Streams video from the onboard camera.

### Intel NUC:
Processes high-load computations for mapping, depth sensing, and robotic control:
- **RPLIDAR Node:** Processes 360-degree laser scans for mapping and obstacle detection.
- **RealSense Depth Camera Node:** Captures depth data for 3D environmental modeling.
- **Interbotix Arm Node:** Controls robotic arm movements and interactions.

## 5. Setting Up Static IP Addresses
### Establish a connection between the NIC and Raspberry Pi
- For wireless connection: On the NUC connect to the Raspberry Pi's hotspot
- For wired connection: Plug in the ethernet cable between the NUC and Raspberry Pi.
 
### Configuring Static IP on Raspberry Pi
Edit the DHCP Client Configuration:
```sh
sudo nano /etc/dhcpcd.conf
```
For wireless connection add:
```sh
interface wlan0
static ip_address=192.168.100.2/24
static routers=192.168.100.1
static domain_name_servers=192.168.100.1
```
For wired (ethernet) connection add:
```sh
interface eth0
static ip_address=192.168.100.2/24
static routers=192.168.100.1
static domain_name_servers=192.168.100.1
```
Apply changes:
```sh
sudo systemctl restart dhcpcd
```

### Configuring Static IP on Intel NUC
Access Network Settings:
```sh
sudo nmtui
```
Set:
- **IPv4 Address:** `192.168.100.3/24`
- **Gateway:** `192.168.100.1`
- **DNS Server:** `192.168.100.1`
Restart Network Manager:
```sh
sudo systemctl restart NetworkManager
```

### Test the connection from Intel NUC to Raspberry Pi:
```sh
ping 192.168.100.2 
```

### Test the connection from Raspberry Pi to Intel NUC:
```sh
ping 192.168.100.3 
```

## 6. Running ROS 2 Across Both Devices
### Setting Environment Variables for ROS 2 Communication:
On **both devices**, set the ROS 2 domain ID: (If you don't set it then it's zero so don't need to set it in this application.... find out with `echo $ROS_DOMAIN_ID`
```
export ROS_DOMAIN_ID=30
```
To make it persistent:
```sh
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```

### Sourcing the ROS 2 Workspace on the NUC
Before launching nodes or sending commands, ensure your ROS 2 workspace is sourced on the NUC. *The Leo Rover's Pi does this already when it turns on, it also runs auto runs nodes as well.*
```sh
source ~/ros_ws/install/setup.bash
```
To avoid doing this manually every time, add it to your `~/.bashrc:`
```sh
echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Sending commands from the NUC to the Pi
Theoretically you should be able to see all active nodes running on both the NUC and Pi from the NUC
```sh
ros2 topic list
```
Also from the NUC you should be able to send commands
```sh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Launching ROS 2 Nodes (Test if it works)
On the Raspberry Pi:
```sh
ros2 launch rover_bringup rover.launch.py
```
On the NUC:
```sh
ros2 launch sensor_bringup sensor.launch.py
```

## 7. Creating a Simple GUI for Leo Rover on NUC (Optional)
Now you can send commands from the NUC you can go about and implement whatever you like. In this step we create a very basic GUI controller for the robot for remote control (if you're using wireless connection). 
### Creating a ROS 2 Node for GUI Control:

A **Graphical User Interface (GUI) controller** can be useful for manually driving the Leo Rover in simulation. This section provides step-by-step instructions for creating a simple GUI using **Tkinter (Python’s standard GUI library)** to publish velocity commands to the rover.

### **Create a New ROS 2 Package**
Navigate to your ROS 2 workspace and create a new package for the GUI controller:
```bash
cd ~/ros2_ws/src
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
cd ~/ros2_ws/src/gui_controller_leo
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



### **Build the `gui_controller_leo` Package**
```bash
cd ~/ros2_ws
colcon build --packages-select gui_controller_leo
source install/setup.bash
```

### **(if required) Clean workspace, ensure all dependencies are installed, and rebuild the workspace**
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

### Run the GUI Controller Node
_Ensure the robot is raised so can't move or is somewhere sensible._
 ```
 ros2 run gui_controller_leo gui_controller
```


## 8. Summary
- **Ensure ROS 2 Humble and a workspace are set up on the NUC.**
- **Follow best practices for workspace organisation.**
- **Leverage DDS for automatic node discovery in ROS 2.**
- **Use static IPs for stable networking.**
- **Create a GUI on the NUC for easy control of the Leo Rover.**
- **Launch ROS 2 nodes on both devices to distribute processing efficiently.**

By following this guide, you establish a distributed ROS 2 system with enhanced usability, organisation, and computational efficiency.

