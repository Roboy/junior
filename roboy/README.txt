README.txt


1. REQUIREMENTS

1.1 Ubuntu (Tested with 12.04 LTS)

1.2 Ubuntu Media Player
sudo apt-get install mplayer2

1.3 Alut libraries
sudo apt-get install libopenal-dev libalut-dev 

1.4 OpenCV for Linux
download from: http://opencv.org/downloads.html
Go through Installation guide: http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation
Install all the packages, e.g. also 
install libgtk2.0-dev and pkg-config
(Git is not necessary)

NOTE: Disable library dc1394 support (it dynamically links libusb which conflicts with the static version of the library from mvBlueFox)
cmake -DWITH_1394=OFF <path to the OpenCV source directory>
make 
sudo make install

IF YOU RUN A VIRTUAL MACHINE: Don't build and install from a shared folder (somehow it does not work)


1.5 Boost thread library
sudo apt-get install libboost-all-dev

1.6 backport of liblog4cplus
sudo apt-add-repository 'deb http://ppa.launchpad.net/launchpad-weyland/roboy/ubuntu precise main'
sudo apt-get update
sudo apt-get install liblog4cplus-dev

1.7 Peak Can USB driver
go to the libs/PeakUSB_CAN/peak-linux-driver-7.7 and do:
make clean
make NET=NO
sudo make install
sudo modprobe pcan 

You can check whether the driver is installed by typing 
cat /proc/pcan 

1.8 MVBlueFOX driver
Depending on your platform, cd into
libs/mvIMPACTx86-64 (64bit) or
mvIMPACT_acquire-x86 (32bit) and run make.

Then cd into Scripts and run
sudo cp 51-mvbf.rules /etc/udev/rules.d/
/etc/init.d/udev restart

1.9 Festival TTS & EST Libraries
sudo apt-get install festival-dev festival libestools2.1 libestools2.1-dev festvox-us1 festvox-en1 festvox-kallpc16k festvox-rablpc16k festvox-us2 festvox-us3

All but openCV in one go:
sudo apt-add-repository 'deb http://ppa.launchpad.net/launchpad-weyland/roboy/ubuntu precise main'
sudo apt-get update
sudo apt-get install mplayer2 libopenal-dev libalut-dev libboost-all-dev liblog4cplus-dev cmake festival-dev festival libestools2.1 libestools2.1-dev libpcan-dev peak-linux-driver-utils peak-linux-driver-dkms build-essential

1.10 ALSA Libraries
sudo apt-get install libasound2-dev

1. DOWNLOAD THE SOURCE CODE

1.1 Get the roboy source files into your current directory:


2. INSTALL THE ROBOY FRAMEWORK

2.1 Create a folder to put the binary files:
mkdir <folder_for_binaries>

2.2 Go into the folder_for_binaries and run:
cmake <path_to_roboy_src_folder>

2.3 Compile the project
make


3. RUN THE ROBOY STATE MACHINE

3.0
added an option in the new configuration file to allow the state machine to run without vision (see roboy/database/configuration/RoboyConfigurationFile.conf)... The only thing you now have to do to be able to run the state machine is to create a local copy of the configuration file (roboy/database/configuration/RoboyConfigurationFile.conf) and change the parameter DATABASE_PATH to the path of your roboy/database directory. You do not need to add this file to the svn.

3.1 Go into <folder_for_binaries>/src/roboy/main
cd <folder_for_binaries>/src/roboy/main

3.2 Run
Run with Configuration File:
[binary_folder]/src/roboy/main/RoboyStateMachine [PATH_TO_YOUR_LOCAL_CONFIGURATION_FILE]
E.g.: (path has to match to where the config file is on your PC)
./RoboyStateMachine ../../../../roboy/database/configuration/RoboyConfigurationFileSerge.conf

Config-Files on the roboy LapTop:
Without the Cameras (in Case they cause crashes):
./RoboyStateMachine ../../../../roboy/database/configuration/ConfigStateMachineNoVision.conf
With the Cameras:
./RoboyStateMachine ../../../../roboy/database/configuration/ConfigStateMachine.conf


you can change the current behaviour by pressing certain keys (check out the GUI for a list).

you should see a line in the prompt with the active state

3.3 For facial expressions
1. Set the beamer screen to 1280x720 in a dual-head setup
2. the -xineramascreen 1 parameter to mplayer should display the video on the robot face.
