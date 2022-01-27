This has been tested on RPi4 with Ubuntu 20.04 server and ROS2 Foxy.

## Setting up a fresh Raspberry Pi

### Ubuntu install

The OS can be flashed onto a fresh SD card according to [this guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#2-prepare-the-sd-card), verbatim.

If a network is needed then [the next section form the same guide](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet)  can be used for that too.

In case that method does not work you can try [this.](https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line)

### Installing ROS 2

Set up a ROS installation as you would usually following the official documentation.

**Package dependency install**
The card swiper package depends on the **rmf_door_msgs** package that you need to install from the [rmf_core repository](https://github.com/osrf/rmf_core). It is possible to install the entire rmf_core package set, but this increases the build time greatly.

### Python requirements

The card wiper package depends also on some Python libraries. Namely **gpiozero** and its subdependency **gpiod**.

Gpiozero can be installed easily via pip. Just install pip via

	sudo apt update && sudo apt install python3-pip
	
Gpiod however needs to be downloaded separately. For that follow the instructions [here](http://abyz.me.uk/rpi/pigpio/download.html).

Once the Python requirements are installed you should be able to build the card swipe package.

### Accessing the Pi pin IO

The Pi pins and their functions don't work out of the box with Ubuntu so some extra setup is necessary.

You should create a systemd service that automatically starts the pin functionality on boot. This can be combined with starting the card swiper package, which is how it will be done in the example.

Create a file in /etc/systemd/system/ called swipe.service and add the following contents.

	[Unit]
	Description="Service to start the swiper node"
	After=network-online.target
	
	[Service]
	Type=simple
	User=ubuntu
	Group=ubuntu
	ExecStart=/home/ubuntu/swipe.sh
	
	[Install]
	WantedBy=default.target
	
When that service is called it runs the file specified in ExecStart. Create this file, in this case in /home/ubuntu/ and add the following contents. "doorname" is a placeholder.

	#!/usr/bin/env/bash
	sudo pigpiod
	source /opt/ros/foxy/setup.bash
	source ~/dev_ws/install/setup.bash
	ros2 run card_swipe_py swiper doorname
	
This starts the pin functionality, sources the ROS workspace and runs the swiper node.

Once the service is created, you can enable it on boot by

	sudo systemctl enable swipe
	
or test it without reboot by

	sudo systemctl start swipe

and check its status any time by

	sudo systemctl status swipe
	
which should show some log messages from the running node.

**NB!!!**

When
	
	sudo pigpiod

has already been run by the service script, killing and restarting the service will show errors. This is expected behaviour and will be fixed on reboot and will not happen on the first run of the service, whether automatically or manually, when the system is booted.

**Error with /dev/mem access**

*This error did not generate when following the official guide for installing Ubuntu 20.04 server on RPi.*

When first setting up the Pi, there might be an issue with accessing the pin inputs/outputs due to permissions not being automatically generated due to using Ubuntu Server instead of Raspbian. In this case do the following:

	sudo groupadd gpio
	sudo usermod -a -G gpio user_name
	sudo grep gpio /etc/group
	sudo chown root.gpio /dev/gpiomem
	sudo chmod g+rw /dev/gpiomem
	
This gives root access to the gpio library.

### PWM output

The output PWM signal is to the GPIO17 pin (6th pin from the top on the inner pin row). A sine wave signal is sent which will actuate the servo from one limit to the other.

-----------------------------------------------------------------------------------------
	
**Ubuntu user**:

	login: ubuntu

	password: covsg24

**Servo wiring**: Purple wire to 5V, blue to GND, grey to GPIO17 (6th pin from the top on the inner pin row).

**Running the created card swiper package from command line:**

Make sure the workspace is sourced with 

	source ~/ros_ws/install/setup.bash
	source /opt/ros/foxy/setup.bash

Once the workspace is sourced you can simply run 
	
	ros2 run card_swipe_py swiper doorname
	
to start the trigger subscriber that will control servo motion based on the received message. There exists a ‘Triggerer’ node that is used for testing and does not need to be run for real application and has not been tested with "doorname" functionality.
