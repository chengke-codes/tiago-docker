# TIAGO-DOCKER

ROS Version: **Kinetic**

A docker-compose project with the [TIAGO Public simulation](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS).

## Prerequisites

Docker installation instructions: https://docs.docker.com/install/

Docker Compose instructions: https://docs.docker.com/compose/install/

Nvidia docker https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html

## NVIDIA settings

For the NVIDIA option this has been tested on the following systems using NVIDIA docker2:

| Ubuntu distribution | Linux Kernel                 | Nvidia drivers |
| ------------------- | ---------------------------- | -------------- |
| 20.04               | 5.4.0<br />5.8.0<br />5.15.0 | 460<br />470   |

## How to install

**The following commands need to be run in local shell (not inside the container)**

1\. Clone this repository. (assume you cloned this into `<tiago_docker-path>`)

2\. `cd <tiago_docker-path>`

3\. `./install.sh`

4\. Append the following commands to your `~/.bashrc` file.

**Don't forget** to replace `<tiago_docker-path>` with the location of where you clone this repository. (The `<tiago_docker-path>` must be a valid absolute path, if you don't know this path, run `pwd` command to show.)

```
export TIAGO_DOCKER_ROOT=<tiago_docker-path>
export PATH=$PATH:$TIAGO_DOCKER_ROOT/bin
```
Then run `source ~/.bashrc` to make the above changes take effect.

5\. Run `tiago_up` to start the container.

6\. Run `tiago_bash` to open a new terminal.

7\. When you want to stop the container, please run `tiago_stop`.

## A Quick demo

1\. Run `tiago_up`

2\. Run `tiago_bash` , then you will enter a internal shell of the container. You can see a **TIAGO** alert in the left side.

3\. Execute `roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium world:=simple_office_with_people`  **inside the container shell**. Gazebo will show up with TIAGO in the following world.
![Screenshot from 2021-03-10 18-35-10.png](https://i.loli.net/2021/03/11/zlb8stkyVwmdMNW.png)



4\. Open a new local terminal then run `tiago_bash`. Then execute `rosrun key_teleop key_teleop.py` **inside** the container shell.

5\. Now you can control TIAGO by using your keyboard.

## Some tips
1\. `<tiago_docker-path>/tiago_home` is **mounted as your home directory** inside the container. Your data that needs to be persisted should be saved there to prevent loss.

2\. If the Gazebo is shown in full-screen without window control, **press** `F11` **twice** to escape.


3\. The container will bind directly to the host's network. The SSH service inside the container will use port **2222**, and the ROS master node will use port **11311** by default. 

4\. If you want to make changes with your ROS environment, you have to edit `.ros_profile` instead of `.bash_rc` in your container. This `.ros_profile` will be loaded by `.bashrc` and the remote python wrapper depends on it.

## Working with PyCharm

1\. Make sure your SSH public/private keys are ready.

2\. Use PyCharm to open your project and then create a remote Python interpreter. 

![Screenshot from 2021-06-13 12-39-44.jpg](https://i.loli.net/2021/06/13/IDM5Rf1CYGyh2iX.jpg)

Python interpreter path should be `/usr/bin/pythonr`

3\. Now you have code hints and auto complete.

## References

1. ROS rocker https://github.com/osrf/rocker
