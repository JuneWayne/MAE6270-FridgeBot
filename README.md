# MAE6270-FridgeBot

Team: Ethan Cao, Eric He, Ray Fang, Katherine Craig

Fridgey is a four-wheel robot controlled through ROS 2.

![Fridgey](miscellaneous/fridgey.jpg)

## Prepare the computers

1. Use Ubuntu with ROS 2 Jazzy on both computers.
2. Put the current project files in `~/human_machine_interface` on both computers.
3. Connect both computers to the same Wi-Fi network.
4. Connect the joystick to the base computer.
5. Connect the Arduino to the robot computer through USB.
6. Set the joystick to its centre position.
7. Stop previous copies of the joystick and motor programs.

For WSL, attach the joystick USB device to Ubuntu before you continue.
The network must permit ROS 2 traffic between the computers.

## Install the required packages

Run once on the **base computer**:

```bash
sudo apt update
sudo apt install ros-jazzy-joy
```

Run once on the **robot computer**:

```bash
sudo apt update
sudo apt install python3-serial
```

## Prepare the Arduino

Skip the upload if the correct program is already installed.

1. Stop `robot_motor.py` before an upload.
2. Close **Serial Monitor**.
3. Close **Serial Plotter**.
4. Install `DualVNH5019MotorShield` through Arduino IDE Library Manager, if necessary.
5. Open `week7_motor_control/week7_motor_control.ino`.
6. Select **Arduino Uno**.
7. Select the Arduino USB port.
8. Click **Upload**.
9. Wait for a successful upload before you continue.

## Check the devices

On the **base computer**, list the joystick devices:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_enumerate_devices
```

Record the joystick ID. Replace `device_id:=0` below if the ID differs.
If the list is empty, correct the device connection or permissions before you continue.

On the **robot computer**, list the serial ports:

```bash
python3 -m serial.tools.list_ports -v
```

Find the Arduino port. Replace `/dev/ttyACM0` below if the port differs.

## Start the system

Use four terminals. Keep each program running.
Use `ROS_DOMAIN_ID=3` in every ROS terminal.

### Robot computer — Terminal 1: motor interface

```bash
cd ~/human_machine_interface
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
python3 robot_motor.py --ros-args -p port:=/dev/ttyACM0
```

Wait for `Ready on /dev/ttyACM0; waiting for /cmd_vel`.
The port name can differ.

### Base computer — Terminal 1: joystick driver

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0 -p deadzone:=0.10
```

The `/joy_node` node publishes joystick input on `/joy`.
The repeat rate keeps messages active while the stick position stays constant.

### Base computer — Terminal 2: velocity commands

```bash
cd ~/human_machine_interface
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
python3 week8_joystick_teleop.py --ros-args --params-file config/week8_joystick.yaml
```

The `/joystick_teleop` node receives `/joy` and publishes `/cmd_vel` at 5 Hz.
The message type is `geometry_msgs/msg/Twist`.

### Robot computer — Terminal 2: message display

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic echo /cmd_vel
```

This terminal displays messages continuously. Terminal 1 controls the motors.

## Check operation

1. Switch on the motor power.
2. Move the stick forward.
3. Move the stick backward.
4. Check that `linear.x` changes.
5. Move the stick left.
6. Move the stick right.
7. Check that `angular.z` changes.
8. Release the stick.
9. Check that both values return to zero.
10. Check that the robot stops.

```text
Joystick -> /joy -> /joystick_teleop -> /cmd_vel
         -> /motor_interface -> USB serial -> Arduino -> motors
```

The USB serial baud rate is 115200.
The current wheel speed request limit is 10 RPM.
Actual wheel RPM is not measured by this program.

## If the system does not operate

| Result | Check |
|---|---|
| Joystick device list is empty | USB connection and input device permissions |
| `/cmd_vel` stays zero | `/joy` input and the joystick axis settings |
| Base computer has changing `/cmd_vel`, but robot has no messages | Wi-Fi connection and ROS settings on both computers |
| Robot receives changing `/cmd_vel`, but motors do not move | Motor interface errors, Arduino upload, and motor power |
| Upload fails | Board selection, port selection, and programs that use the serial port |

If the Arduino port gives `Permission denied`, run on the **robot computer**:

```bash
sudo usermod -aG dialout "$USER"
```

Log out of the robot computer. Log in again before you start the motor interface.

## Stop the system

1. Release the stick.
2. Check that the robot stops.
3. Press **Ctrl+C** in Base Terminal 2.
4. Press **Ctrl+C** in Base Terminal 1.
5. Press **Ctrl+C** in Robot Terminal 1.
6. Press **Ctrl+C** in Robot Terminal 2.
7. Switch off the motor power.
