# SLAM-for-Autonomous-Robots

# Autonomous Systems:

Autonomous Systems are those systems that are developed to function without the interference of humans and also have the added benefit of working with humans (like HRC - Human-Robot Collaboration). These systems have semantic understanding, which makes them semantically understand static objects and dynamic objects in the environment.

Examples of autonomous systems are Robots deployed in various industries for instance Rovers for space exploration, Kiva - amazon developed robots for warehouses, robots for deliveries, and the most notable and recognized self-driving cars by Tesla and Google.

To achieve a system that works autonomously. Several algorithms and models are working in its embedded system. Here, I am glad to work on one of the algorithms, that is crucial and used widely in these systems.

# SLAM - Simultaneous Localization and Mapping

SLAM is a concept, where an autonomous system is provided with the ability to navigate. At the same time, it can locate itself in the environment where it is placed with little or no human intervention.

# Sensor Selection:

I have a selection of sensors for this project, but they have their disadvantages on using them  in indoor environment.

1. Reflection and shadows of metal surfaces, Glass
2. Bright Lights can cause mis-reading for camera used for detection of objects.
3. Problems with the range of the sensor (1m, 2m).

Here's a list of sensor, which I'm planning to simulate in Isaac Sim to see how it behaves when exposed to reflections, bright lights, range problems.

1. LiDar (range is good, works to some extent with reflections, works in low lighting); (poor for detecting dynamic objects)
2. Camera (detects and segregates moving humans, static objects, walls); (reflection, low lighting, even overly bright lighting)
3. Encoders 

#### <B>** I am using Ubuntu 22.04 to run Isaac Sim and ROS2 <B>

# Installation of Nvidia Cuda cores: (If updated already, skip this step)

Link to find Cuda: https://developer.nvidia.com/cuda-downloads

#### Click the above link and follow below: <B>** Currently I'm using Cuda Toolkit 12.8 <B>
1. Select the operating system.
2. Select the archietecture.
3. Select the windows version.
4. Select the installer type and download.
#### ----- Windows user can downloaded at step 2----- continue to step 3 for Linux Users---------
5. Select your distribution (Fedora, Ubuntu, Debian, so on).
6. Select the version which you are using.
7. Finally, the Installer type.

#### Now , you will get the Installation Instruction, Follow that. <B>Else Follow the Steps wriiten below:<B>

## Step - 1: open terminal and run this code one by one:
- wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
- sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
- wget https://developer.download.nvidia.com/compute/cuda/12.8.0/local_installers/cuda-repo-ubuntu2204-12-8-local_12.8.0-570.86.10-1_amd64.deb
- sudo dpkg -i cuda-repo-ubuntu2204-12-8-local_12.8.0-570.86.10-1_amd64.deb
- sudo cp /var/cuda-repo-ubuntu2204-12-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
- sudo apt-get update
- sudo apt-get -y install cuda-toolkit-12-8

## Step - 2: Driver Installation: 
- Select any one option, either open kernel or legacy kernel module flavor
#### Open Kernel Module Flavor:
- sudo apt-get install -y nvidia-open
#### Legacy Kernel Module Flavor:
- sudo apt-get install -y cuda-drivers

# Updating Cuda Deep Neural Network Cores (CudaNN)

Link to find CudaNN: https://developer.nvidia.com/cudnn-downloads

## Follow the steps 1-7 mentioned above in Cuda Core Installation untill you get the Installation Instruction. Once you reach here, Follow Below: <B>** Currently I'm using Cuda Toolkit 9.7.1<B>

1. Click the download button and you will see .deb file downloading.
2. open Terminal and carefully, Choose the directory where the file is downloaded. Write these one by one:
- wget https://developer.download.nvidia.com/compute/cudnn/9.7.1/local_installers/cudnn-local-repo-ubuntu2204-9.7.1_1.0-1_amd64.deb
- sudo dpkg -i cudnn-local-repo-ubuntu2204-9.7.1_1.0-1_amd64.deb
- sudo cp /var/cudnn-local-repo-ubuntu2204-9.7.1/cudnn-*-keyring.gpg /usr/share/keyrings/
- sudo apt-get update
- sudo apt-get -y install cudnn

3. Correctly specify the version of Cuda cores that you installed in the below code:
sudo apt-get -y install cudnn-cuda-12

<B>----- Cores are updated -----<B>