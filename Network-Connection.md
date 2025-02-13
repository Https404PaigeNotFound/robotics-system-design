# Integrating an Intel NUC with Leo Rover

This guide expands on the **LeoOS and ROS 2 setup** by integrating an **Intel NUC** with the **Leo Rover's Raspberry Pi**. This enhances computational capabilities, enables efficient processing distribution, and optimizes performance by running resource-intensive tasks on the NUC.

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
- **Optimized ROS 2 Performance:** Launch and manage ROS 2 nodes across both devices for coordinated execution.
- **Automatic Node Discovery:** ROS 2 uses **Data Distribution Service (DDS)** to enable seamless discovery of nodes across the network without additional configuration.

## 3. Best Practices for Workspace Layout
When organizing custom ROS 2 nodes, it's best to structure them by function. A typical ROS 2 workspace (`ros2_ws`) should look like this:
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
### Configuring Static IP on Raspberry Pi:
```sh
sudo nano /etc/dhcpcd.conf
```
Add:
```sh
interface wlan0
static ip_address=192.168.100.2/24
static routers=192.168.100.1
static domain_name_servers=192.168.100.1
```
Apply changes:
```sh
sudo systemctl restart dhcpcd
```

### Configuring Static IP on Intel NUC:
```sh
sudo nmtui
```
Set:
- **IPv4 Address:** `192.168.100.3/24`
- **Gateway:** `192.168.100.1`
- **DNS Server:** `192.168.100.1`
Restart network:
```sh
sudo systemctl restart NetworkManager
```

## 6. Running ROS 2 Across Both Devices
### Setting Environment Variables for ROS 2 Communication:
On **both devices**, set the ROS 2 domain ID:
```sh
export ROS_DOMAIN_ID=30
```
To make it persistent:
```sh
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```

### Launching ROS 2 Nodes
On the Raspberry Pi:
```sh
ros2 launch rover_bringup rover.launch.py
```
On the NUC:
```sh
ros2 launch sensor_bringup sensor.launch.py
```

## 7. Creating a Simple GUI for Leo Rover on NUC
### Creating a ROS 2 Node for GUI Control:
1. **Install dependencies:**
   ```sh
   sudo apt install python3-pyqt5 ros-humble-rqt ros-humble-rqt-gui
   ```
2. **Create a new ROS 2 package:**
   ```sh
   ros2 pkg create leo_gui --build-type ament_python --dependencies rclpy std_msgs
   ```
3. **Create a GUI script (`gui.py`):**
   ```python
   import sys
   from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
   import rclpy
   from geometry_msgs.msg import Twist

   class RoverGUI(QWidget):
       def __init__(self, node):
           super().__init__()
           self.node = node
           self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
           self.initUI()

       def initUI(self):
           layout = QVBoxLayout()
           forward_btn = QPushButton('Move Forward')
           forward_btn.clicked.connect(self.move_forward)
           layout.addWidget(forward_btn)
           self.setLayout(layout)

       def move_forward(self):
           msg = Twist()
           msg.linear.x = 0.5
           self.publisher.publish(msg)

   def main():
       rclpy.init()
       node = rclpy.create_node('rover_gui')
       app = QApplication(sys.argv)
       ex = RoverGUI(node)
       ex.show()
       sys.exit(app.exec_())

   if __name__ == '__main__':
       main()
   ```
4. **Run the GUI:**
   ```sh
   ros2 run leo_gui gui.py
   ```

## 8. Summary
- **Ensure ROS 2 Humble and a workspace are set up on the NUC.**
- **Follow best practices for workspace organization.**
- **Leverage DDS for automatic node discovery in ROS 2.**
- **Use static IPs for stable networking.**
- **Create a GUI on the NUC for easy control of the Leo Rover.**
- **Launch ROS 2 nodes on both devices to distribute processing efficiently.**

By following this guide, you establish a distributed ROS 2 system with enhanced usability, organization, and computational efficiency.

