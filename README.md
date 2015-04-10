# rosbeam
A ROS driver for BeamPro

## Installation

Dependency:
Boost (libboost-all-dev on Debian/Ubuntu), cmake, make, python and virtualenv, gcc 4.9 (tested version)

Build instructions:
````
cd build
make 
sudo make install
````

This will generate a tarball `overlay.tgz` inside the `build` directory. `sudo` here will not touch anything outside the `build` directory. It is only for installing root UID and setuid.

## Deployment

* Power off Beam
* Extract the SD card from the back of Beam
* Mount it (showing as "RPD-STORE" on Ubuntu, or use `mount /dev/mmcblk0p2 /mnt` on Debian)
* Configure wifi_dev_mode `touch /mnt/config/wifi_dev_mode`
* `cp overlay.tgz /mnt/images/`
* Edit `/mnt/config/release/target` to add `- /store/images/overlay.tgz` as the last "layer".
* Unmount the SD card and put it back.

## Running

* You can connect to Beam with an Ethernet cable. The network should be configured as 192.168.68.2/255.255.255.0. Beam will be 192.168.68.1.
* `export ROS_HOSTNAME=192.168.68.2`
* `roscore`
* `ssh st@192.168.68.1 rosbeam-bridge.sh`, password is st.

Now you will be able to see ROS topics.

## Uninstalling

ssh to Beam, and `rm /store/config/wifi_dev_mode` will disable all modifications.

`rm /store/images/overlay.tgz` and remove `overlay.tgz` from `/store/config/release/target` will completely remove all modidications.
