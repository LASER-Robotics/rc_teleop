# rc_teleop
ROS package that allow control of gazebo drone using RC transmitter. It remaps the topic ~joy to ~joy_raw, remaps the channels defined in the launch file and publishes to ~joy. The launchs and nodes can be used standallone, but the tmux scripts use the [MRS system](https://github.com/ctu-mrs/mrs_uav_system) (Ubuntu 20.04).

## Installing

Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). If you already have one, clone the rc_teleop package to its src folder. 

```bash
cd $your_workspace_path/src
git clone https://github.com/LASER-Robotics/rc_teleop.git
```

Then, build the package with:

```bash
cd $your_workspace_path
catkin build
```

## Running the examples

Use the tmux sessions in scripts_tmux. Choose the appropriate joystick.

```bash
cd $your_workspace_path/scripts_tmux/teleop_logitec
./start.sh
```

You can also run any of the mrs [examples](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts), stop the *roslaunch mrs_uav_general logitech_joystick.launch* in tab *easy_control* (or remove the roslaunch line from session.yml) and run the appropriate roslaunch:

```bash
roslaunch rc_teleop rc_teleop_txs.launch
roslaunch rc_teleop rc_teleop_logitec.launch
roslaunch rc_teleop rc_teleop_flysky.launch
```

## Using

Wait for the drone to takeoff. The sticks are in **MODE 1** (left-vertical pitch; left-horizontal yaw; right-vertical altitude, right-horizontal roll). The control is similar to [Altitude mode](https://docs.px4.io/main/en/flight_modes/altitude_mc.html).

### Logitec

Press button **L2** for manual control.

### TX16S

On the radio transmitter, configure the button SF to Channel 5. Press **SF** for manual control. 

### Flysky

On the radio transmitter, configure the button SWA to Channel 5. Press **SWA** for manual control.
