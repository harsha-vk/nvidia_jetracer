# JetRacer

JetRacer is an autonomous RC racer car application which runs on NVIDIA Jetson Nano.

## Setup

To get started with JetRacer, follow these steps

1. Order parts from the [bill of materials](https://github.com/NVIDIA-AI-IOT/jetracer/blob/master/docs/latrax/bill_of_materials.md).
    > Preferred Camera = [IMX219-83 Stereo Camera](https://www.waveshare.com/imx219-83-stereo-camera.htm)</br>
    > PWM Servo Motor Driver + RC Servo Multiplexer = Arduino Pro Mini 3.3v

2. Follow the hardware setup from [this link](https://github.com/NVIDIA-AI-IOT/jetracer/blob/master/docs/latrax/hardware_setup.md).
    > Arduino - Multiplexer [setup](assets/multiplexer/multiplexer.md)

3. Follow the software setup [below](#Software-Setup).

### Software Setup

1. Download Jetson Nano Developer Kit SD Card Image (version: 4.5) from [this link](https://developer.nvidia.com/embedded/downloads).

2. Follow [these instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) to install SD Card Image on Jetson Nano.

3. Build Jetson-Inference from source by following this [link](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md).

4. Install ROS2 Eloquent Elusor Desktop by following instructions below.

    ```bash
    sudo apt-get update && sudo apt-get install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt-get update && sudo apt-get install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

    sudo apt-get update
    sudo apt-get install ros-eloquent-desktop python3-colcon-common-extensions python3-rosdep python3-opencv
    sudo apt-get install python3-pip
    pip3 install -U argcomplete

    sudo rosdep init
    rosdep update

    echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
    ```
5. VNC configuration for Nvidia Jetson Nano is [here](assets/vnc_config.md).