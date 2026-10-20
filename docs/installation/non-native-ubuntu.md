# Non-native Ubuntu

The software needs Ubuntu 22.04 and ROS 2 Humble. On Ubuntu 22.04, start with [Kinova drivers](kinova-drivers.md). On another system, use one of the environments below.

| Host | Environment |
|---|---|
| Linux other than Ubuntu 22.04 | Distrobox |
| Windows | WSL 2 or a virtual machine |
| macOS | Virtual machine |

## Distrobox

[Distrobox](https://distrobox.it/) runs an Ubuntu 22.04 container that shares the home folder, the network and the display of the host.

1. Install Distrobox with the package manager of the host. On Fedora:

    ```bash
    sudo dnf install distrobox
    ```

2. Create the container and enter it:

    ```bash
    distrobox create --name ubuntu-22-04 --image ubuntu:22.04
    distrobox enter ubuntu-22-04
    ```

3. Check the version inside the container:

    ```bash
    cat /etc/os-release
    ```

    The output contains `VERSION_ID="22.04"`.

Enter the container again with the second command of step 2.

## WSL 2

1. In PowerShell as administrator, install Ubuntu 22.04. Restart Windows if asked.

    ```powershell
    wsl --install -d Ubuntu-22.04
    ```

2. Check that the distribution uses WSL 2:

    ```powershell
    wsl --list --verbose
    ```

    `Ubuntu-22.04` has the version `2`.

3. Start Ubuntu with `wsl -d Ubuntu-22.04` or from the Start menu.

Keep the ROS 2 workspace in the Linux filesystem, for example `~/workspace`. A workspace under `/mnt/c` builds slowly. RViz runs through WSLg. Check that `echo $DISPLAY` is not empty.

## Virtual machine

Install Ubuntu Desktop 22.04 LTS in a virtual machine such as UTM, Parallels, VMware or VirtualBox. Give it 4 or more cores, 8 GB or more of memory, 40 GB or more of storage and 3D acceleration. Keep the workspace inside the machine, not in a shared folder.

## USB devices

The Kinect needs a USB device inside the environment. Windows and a virtual machine do not pass a device on by default. Attach it with USB/IP in WSL, for example with `usbipd-win`, or with the USB settings of the virtual machine. Check the device inside Ubuntu:

```bash
lsusb
```

## Check the environment

| Check | Command | Expected |
|---|---|---|
| Ubuntu version | `cat /etc/os-release` | `VERSION_ID="22.04"` |
| Python | `python3 --version` | 3.10 or later |
| Git | `git --version` | A version number. |
| Internet | `ping -c 3 ubuntu.com` | Replies. |
| Display for RViz | `echo $DISPLAY` | Not empty. |

Next: [Kinova drivers](kinova-drivers.md).
