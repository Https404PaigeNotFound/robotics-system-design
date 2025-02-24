# Installing LeoOS, Remote Access, and Updating. 

This guide covers installing LeoOS on a Raspberry Pi, setting up remote access, and updating the system.

## 1. Install Required Software

### Download and Install:
- **LeoOS**: [Download latest full version](https://github.com/LeoRover/LeoOS-ros2/releases)
- **Balena Etcher**: [Download](https://etcher.balena.io) (for burning LeoOS to SD card)
- **PuTTY**: [Download](https://www.putty.org) (*Windows users only, for SSH/remote access)

## 2. Flash LeoOS to SD Card
1. Insert SD card into your computer.
2. Open Balena Etcher.
3. Select the downloaded LeoOS image.
4. Choose your SD card and click **Flash** (takes ~5 minutes).

## 3. Powering On LeoRover
1. Insert the SD card into the Raspberry Pi.
2. Press the power button on LeoRover.
3. Wait for the green LED to blink (~15-20 sec) until the Wi-Fi network appears.

### Preventing Wi-Fi Conflicts
1. Only turn on **one** LeoRover at a time.
2. Connect to its default Wi-Fi network.
3. Change the network ID to **LeoRover-GROUPX** (X = your group number).
4. Other groups should repeat the process.

## 4. Connecting to LeoRover Using SSH
*SSH (Secure Shell) is a remote access protocol that allows you to control the Raspberry Pi from another device via a command-line interface. It is lightweight, secure, and ideal for running commands, configuring settings, and managing files without a graphical interface.*
### Windows:
1. Connect to LeoRover Wi-Fi (default password: `password`).
2. Open PuTTY, enter IP: `10.0.0.1`, and click **Open**.
3. Accept the security warning.
4. Login:
   - **Username**: `pi`
   - **Password**: `raspberry` (invisible while typing).

### Linux:
1. Open terminal.
2. Run:
   ```
   ssh pi@10.0.0.1
   ```
3. Enter **yes** and **raspberry** when prompted.

## 5. Change Wi-Fi ID
1. In PuTTY or SSH, enter:
   ```sh
   sudo nano /etc/hostapd/hostapd.conf
   ```
2. Modify **SSID** to `LeoRover-GROUPX`.
3. Save changes (`Ctrl+O`, `Enter`, `Ctrl+X`).
4. Restart network:
   ```sh
   sudo systemctl restart hostapd
   ```

## 6. Remote Desktop Access
*A Remote Desktop Connection provides graphical access to the Raspberry Pi’s desktop environment from another device. It allows you to interact with the system visually, making it useful for GUI-based applications, file management, and debugging.*
### Windows + PuTTY:
1. Open **Remote Desktop Connection**.
2. Enter IP: `10.0.0.1` and connect.
3. Accept warning.
4. Login:
   - **Username**: `pi`
   - **Password**: `raspberry`

### Linux:
1. Open **Remmina**.
2. Click **Add New Remote**.
3. Enter:
   - **IP**: `10.0.0.1`
   - **Username**: `pi`
   - **Password**: `raspberry`
5. Click **Connect**.

## Internet & Updating LeoRover

### 1. Connect to the Internet (UoM Wi-Fi)
1. Open Network Manager:
   ```sh
   nmtui
   ```
2. Select **Activate a connection** > **UoM_Wifi**.
3. Enter login details.

### 2. System Update & Upgrade 
```sh
sudo apt update && sudo apt upgrade
```

- May prompt a confirmation at ~72%. Press `n` and `Enter`.

### 3. Update Firmware 
```sh
ros2 run leo_fw update
```
- Select `1) LeoCore`.
- Robot's LED should stop blinking after update.

## 4. Driving the Robot
1. Open a browser and enter `10.0.0.1`.
2. Use the interface to control the robot.

## 5. Test ROS Installation
### View active topics:
```sh
ros2 topic list
```
### Output data from topics e.g. joint states or imu:
```sh
ros2 topic echo /joint_states
```

```sh
ros2 topic echo /firmware/imu
```
### Send movement command (place robot on the ground first!):
*To ensure continous movement use  `--rate 10` which publishes the command at a given frequence, in this example 10Hz. You can send one move command by replacing `--rate 10` with `--once`*
```sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
### Send stop moving command:
```sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Now, you can navigate using the browser interface or ROS commands!

