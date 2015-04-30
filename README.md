# rosbeam
A ROS driver for BeamPro

## NOTICE

This repository was established purely for the purpose of knowledge dissemination and access to open source.  This repository will not be actively maintained.  We provide no promise of responding to issues and pull requests submitted to this repository.

## WARNING

In addition to the disclaimers above and in LICENSE, note that this work is experimental development on open source systems for the purposes of academic research. Please take extra precautions during operation.

## Installation

Dependency:
Boost (libboost-all-dev on Debian/Ubuntu), cmake, make, python and virtualenv (`apt-get install virtualenv` or `apt-get install python-virtualenv` or `pip install virtualenv`), gcc 4.8 (tested version)

Build instructions:
````
cd build
make 
sudo make install
````
During `make`, there can be git checkout errors caused by network interruptions. Rerun `make` to continue.

This will generate a tarball `overlay.tgz` inside the `build` directory. `sudo` here will not touch anything outside the `build` directory. It is only for installing root UID and setuid.

`make clean` will clear out temporary build files. `make cleanall` will remove the cache of downloaded sources.

## Deployment

* Power off Beam
* Extract the SD card from the back of Beam
* Mount it (showing as `/media/$USERNAME/RPD-STORE` on Ubuntu, or manually `mount /dev/mmcblk0p2 /mnt` on Debian)
* Configure wifi_dev_mode `sudo touch /mnt/config/wifi_dev_mode`
* `sudo cp overlay.tgz /mnt/images/`
* Edit `/mnt/config/release/target` to add `- /store/images/overlay.tgz` as the last "layer".
* Unmount the SD card and put it back.

## Running

* You can connect to Beam with an Ethernet cable. The network should be configured as 192.168.68.2/255.255.255.0. Beam will be 192.168.68.1.
* `export ROS_HOSTNAME=192.168.68.2 ROS_MASTER_URI=http://192.168.68.2:11311`
* `roscore`
* `ssh st@192.168.68.1 rosbeam-bridge.sh`, password is st.

Now you will be able to see ROS topics.

With `python-pygame` installed, you can drive Beam with `python drive.py` (leave the cursor in the window during driving).

Controlling Beam over WIFI is not recommended but possible.
* First find out the IP address of Beam `$BEAM_WIFI_IP` over WIFI: `ssh st@192.168.68.1 ip addr`
* Restart the bridge: `ssh st@$BEAM_WIFI_IP 'pkill rosbeam-bridge && rosbeam-bridge.sh'`

## Uninstalling

ssh to Beam, and `rm /store/config/wifi_dev_mode` will disable all modifications.

`rm /store/images/overlay.tgz` and remove `overlay.tgz` from `/store/config/release/target` will completely remove all modidications.
