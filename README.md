This has been tested on RPi4 with Ubuntu 20.04 server and ROS2 Foxy.

**Interacting with the RPi**: Since the card swiper has not been set to start on boot, the RPi has to be accessed via SSH or other means. To setup wifi for ssh follow this guide: [https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet) 

Edit: Or follow [this](https://iot.samteck.net/raspberry-pi/setting-up/wifi-raspberry-pi-ubuntu-headless/) 

**Ubuntu user**:

	login: ubuntu

	password: covsg24

**Servo wiring**: Purple wire to 5V, blue to GND, grey to GPIO17 (6th pin from the top on the inner pin row).

**Running the created card swiper package:**

Two ros2 packages are on the RPi, one contains the _rmf_door_msgs_ and the other is the one I created called _card_swipe_py_, which right now needs the _rmf_door_msgs_ for message type definition. To run the test, firstly source the workspace with ‘_source ~/ros_ws/install/setup.bash’_. The ros distro is sourced automatically with ~/.bashrc.

Once the workspace is sourced you can simply run ‘_ros2 run card_swipe_py triggerer_’ to start the _trigger_publisher.py _script and ‘_ros2 run card_swipe_py swiper’ _to start the trigger subscriber that will control servo motion based on the received message.

Currently the ‘_triggerer’ _publishes the DoorMode message with DoorMode.value of 2 and 1 intermittently. Value 2 makes the servo swipe back and forth at a reasonable speed using a sine wave control signal, and value 1 makes the servo return to default position.
