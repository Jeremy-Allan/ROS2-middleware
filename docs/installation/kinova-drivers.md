# Kinova drivers

The middleware moves the arm through the Kinova ROS 2 packages, `ros2_kortex`. This page installs them and checks them with simulated hardware. See [Kinova drivers and APIs](../technical/kinova-drivers-and-apis.md) for what the packages provide.

The [setup script](setup-script.md) does the steps of this page and of [Middleware and proxy](middleware-and-proxy.md) in one command. It pins `ros2_kortex` to a tested commit instead of the tip of `humble`.

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble. See the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- Git, the ROS 2 build tools `colcon` and `rosdep`, and the `ros2 control` command:

```bash
sudo apt install git ros-dev-tools ros-humble-ros2controlcli
```

## Install the packages

1. Load ROS 2 in the terminal and check the distribution:

    ```bash
    source /opt/ros/humble/setup.bash
    echo $ROS_DISTRO
    ```

    The output is `humble`.

2. Create the workspace and clone the `humble` branch of `ros2_kortex`:

    ```bash
    mkdir -p ~/workspace/ros2_kortex_ws/src
    cd ~/workspace/ros2_kortex_ws/src
    git clone -b humble --single-branch https://github.com/Kinovarobotics/ros2_kortex.git
    ```

3. Install the dependencies. Run `sudo rosdep init` once for each machine.

    ```bash
    cd ~/workspace/ros2_kortex_ws
    sudo rosdep init
    rosdep update
    rosdep install --ignore-src --from-paths src -y -r
    ```

4. Build the workspace. `--parallel-workers 3` limits the memory use of the build.

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
    ```

5. Load the workspace in every terminal that uses it:

    ```bash
    source ~/workspace/ros2_kortex_ws/install/setup.bash
    ```

## Check the installation

List the packages:

```bash
ros2 pkg list | grep -E "kortex|kinova"
```

The list contains these packages:

```text
kinova_gen3_6dof_robotiq_2f_85_moveit_config
kinova_gen3_7dof_robotiq_2f_85_moveit_config
kinova_gen3_lite_moveit_config
kortex_api
kortex_bringup
kortex_description
kortex_driver
```

Start the driver with simulated hardware. `robot_ip` is a required argument and has no effect with simulated hardware.

```bash
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true
```

RViz opens and shows the arm. In a second terminal, list the controllers:

```bash
ros2 control list_controllers
```

`joint_state_broadcaster`, `joint_trajectory_controller` and `gen3_lite_2f_gripper_controller` are `active`. `twist_controller` is `inactive`.

## Connect to the arm

1. Connect the arm to the computer with an Ethernet cable. All arms ship with the address `192.168.1.10`. With a USB to Ethernet interface instead of USB over RNDIS, the address is `192.168.2.10`.
2. Give the Ethernet interface of the computer a static address in the subnet of the arm, for example `192.168.1.11` with the netmask `255.255.255.0`.
3. Check the connection:

    ```bash
    ping -c 3 192.168.1.10
    ```

    All three requests are answered.

To start the middleware against the arm, see [Simulation vs real arm](simulation-vs-real.md).

Next: [Middleware and proxy](middleware-and-proxy.md).
