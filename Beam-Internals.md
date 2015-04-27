# Beam Internals

## Filesystem structures

The SD card in Beam stores everything that Beams run on. Its partition layout is this:

```
$ sudo parted /dev/mmcblk0 print
Model: SD SP04G (sd/mmc)
Disk /dev/mmcblk0: 3964MB
Sector size (logical/physical): 512B/512B
Partition Table: msdos

Number  Start   End     Size    Type     File system  Flags
 1      32.3kB  987MB   987MB   primary  ext4         boot
 2      987MB   2969MB  1982MB  primary  ext4
 3      2969MB  3956MB  987MB   primary  ext4
```

After default mounting on Ubuntu 14.04 trusty, the structure looks like this:
```
$ lsblk /dev/mmcblk0
NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
mmcblk0     179:0    0   3.7G  0 disk 
├─mmcblk0p1 179:1    0 941.3M  0 part /media/obot/RPD-BOOT
├─mmcblk0p2 179:2    0   1.9G  0 part /media/obot/RPD-STORE
└─mmcblk0p3 179:3    0 941.3M  0 part /media/obot/RPD-LOG
```

where `RPD-BOOT` partition stores the bootstrap files, `RPD-STORE` stores application files, and `RPD-LOG` stores logs.

### RPD-BOOT

The boot partition contains the bootloader grub, the kernel `bzImage`, and the initial squashfs filesystem `main-0b182d3da87e-1425884538.474827051.sqsh`.

```
$ ls -l RPD-BOOT/
total 87824
drwxr-xr-x 3 root root     4096 Apr 22  2014 boot
lrwxrwxrwx 1 root root       33 Apr 12 23:15 bzImage -> bzImage-0b182d3da87e-4883417f9a50
-rw-r--r-- 1 root root  6138832 Mar  9 03:05 bzImage-0b182d3da87e-4883417f9a50
lrwxrwxrwx 1 root root       33 Apr 15 09:51 bzImage.last -> bzImage-0b182d3da87e-4883417f9a50
drwxr-xr-x 2 root root     4096 Nov 19 01:58 grub
drwx------ 2 root root    16384 Apr 22  2014 lost+found
-rw-r--r-- 1 root root 83767296 Mar  9 03:05 main-0b182d3da87e-1425884538.474827051.sqsh
```

The path of the initial filesystem seems hardcoded in the kernel. Its content can be extracted with squashfs-tools:
```
$ unsquashfs -d /tmp/main main-0b182d3da87e-1425884538.474827051.sqsh
...
$ ls /tmp/main
bin  boot  dev  etc  home  lib  lib64  media  mnt  opt  proc  root  run  sbin  selinux  srv  store  sys  tmp  usr  var
$ cat /tmp/main/etc/issue.net 
Ubuntu 12.04 LTS
```

### RPD-STORE

```
$ ls -l RPD-STORE/
total 28
drwxr-xr-x 9 root root  4096 Apr 14 15:20 config
drwxr-xr-x 2 root root  4096 Apr 20 17:43 images
drwx------ 2 root root 16384 Apr 22  2014 lost+found
```

In `config` directory, there are all kinds of configurations. The ones relevant to us are

```
$ cat RPD-STORE/config/release/target 
kernel:
  archive: /store/images/base-kernel-0b182d3da87e-955490efbca4.tar
  file: bzImage-0b182d3da87e-4883417f9a50
  main: main-0b182d3da87e-1425884538.474827051.sqsh
  version: '#ST-0b182d3da87e SMP PREEMPT Mon Mar 9 00:02:53 PDT 2015'
layers:
- /store/images/system-5d77b1782a85-2105f4d9f823.tgz
- /store/images/software-0a86e7acbc7f-be572326600d.tgz
no_watchdog: false
release_id: !!python/unicode 'software-0a86e7acbc7f-be572326600d'
unstable: false
```

And a potential `RPD-STORE/config/wifi_dev_mode` which activates development mode if created.

The release target file specifies which images to extract to create the application chroot filesystem. `/media/RPD-STORE` is mounted as `/store` on Beam.

The `base-kernel-` image file is extracted to the RPD-BOOT partition during updates.
```
$ tar tf base-kernel-0b182d3da87e-955490efbca4.tar 
./
./main-0b182d3da87e-1425884538.474827051.sqsh
./boot/
./boot/grub/
./boot/grub/grub.cfg
./boot/grub/theme/
./boot/grub/theme/beam.png
./bzImage-0b182d3da87e-4883417f9a50
```

The `system-` and `software-` "layers" are extracted into a directory to create the application chroot filesystem.

## Booting process

### Bootloader

After POST, BIOS loads GRUB from MBR. Although it seems the platform is capable of UEFI booting. When the SD card is not plugged in before booting, it will drop into a TianoCore EFI shell.

GRUB is configured in RPD-BOOT/boot/grub/grub.cfg to boot up the kernel with plain parameters:
```
menuentry 'RPD Default Image' --class os {
  strecordfail
  linux /bzImage loglevel=3 splash
}
```

The kernel will somehow load the "main" squashfs as the root filesystem.

### Base Ubuntu system

The kernel will load a standard Ubuntu 12.04 precise userspace, and execute `/sbin/init` which is Upstart. Upstart will follow standard Ubuntu booting procedures, until the custom `st.conf` where the booting process enters Suitable's modification:
```
$ cat /tmp/main/etc/init/st.conf 
# chroot - run the chroot environment according to our symlink
#

description	"run the chroot environment"

exec tofile /var/log_permanent/st/init-base.log init-base

post-stop exec tofile /var/log_permanent/st/init-base.log cleanup-base
```
Here `tofile` is a trivial script at `/usr/sbin/tofile` which basically redirects the output of the command to a file. `init-base` (`/usr/sbin/init-base`) is the entry point to the application filesystem.

### `init-base`

This script sets up the application chroot filesystem created by Suitable.

Several filesystems have been mounted according to fstab:
```
$ cat /tmp/main/etc/fstab
...
LABEL=RPD-BOOT /boot ext4 defaults 0 0
LABEL=RPD-STORE /store              ext4  defaults            0 0
LABEL=RPD-LOG   /var/log_permanent  ext4  defaults            0 0
```

* First, parse the YAML configuration at `/store/config/release/target`, verify kernel versions.
* Extract all layers in the order specified in the release target file to the chroot filesystem `/mnt/running`. For `.tgz` files, use `tar -xzf` to extract; for directories, just `cp -a`; for other types of files report errors.
* Run `/mnt/running/mount` to set up necessary mount points for the chroot filesystem.
* Run `/mnt/running/init` to enter the chroot filesystem: `chroot /mnt/running $RPD_ROOT/run`.

### `$RPD_ROOT/run`

Inside the chroot, the original `/mnt/running` becomes the new root filesystem, and anything outside the original `/mnt/running` is supposed to be unaccessible unless mounted otherwise. The log during this phase is to be found at `/var/log_permanent/st/init-st.log` in the `RPD-LOG` partition.

RPD_ROOT is an environment variable read from `/mnt/running/home/st/sw-dev/install/env/paths.sh` during `/mnt/running/init`. Its value is `/home/st/sw-dev/install`. Here are the most of Suitable's application files.

The `run` script will perform various setup procedures. Interesting parts include:
* Set up the network interface `wan0` on the Ethernet port, if the file `/store/config/wifi_dev_mode` exists. The IP address of Beam is set up 192.168.68.1, and the gateway address is set to 192.168.68.2.
* Set up the iptables firewall. Reject almost everything on `wlan0` and `wlan1`.
* Start X server `su st -l -c "xinit /etc/xinitrc -- /usr/bin/Xorg vt8 -novtswitch $nocursor"&`
* Configure LD_LIBRARY_PATH (probably make it `LD_LIBRARY_PATH=/home/st/sw-dev/install/bin`)
* Set binary permissions: add some capabilities to several binaries, and set uid to "st" for almost all binaries in `/home/st/sw-dev/install/bin`
* Execute `$RPD_ROOT/st-run`, which starts `$RPD_ROOT/scripts/texspawner`

### `texspawner`

Beam was originally called Texai. This texspawner script spawns `/home/st/sw-dev/install/bin/texclient` and restarts it when crashing unexpectedly, or performs cleanup if errors are considered too severe.

`texclient` is the main application program controlling Beam's driving and communication.

## Gaining root access to Beam

Inside the chroot, SSH server will be started on `wan0` interface port 22 by `scripts/rpd_setup.py` if wifi_dev_mode is set up. But under the default settings, password login is not allowed:

```
$ grep ^Password software-0a86e7acbc7f-be572326600d/etc/ssh/sshd_config 
PasswordAuthentication no
```
And the password itself is not known.
In `scripts/rpd_setup.py`, `ssh_setup_user` uses `usermod` to reset the password for user "st" to a certain hash (not shown here) with the cleartext not known. If we can remove that setting, and change the password, we can then gain SSH access to Beam by just `ssh st@192.168.68.1`.

To enable password login and replace the password to known hash, or to perform any needed changes, there are two methods:
* Directly replace the content of the compressed images to our version
* Create an additional layer in the release target to overwrite certain scripts along the startup process to change the content to our version

Because there are frequent updates to Beam's images, it is imperative to minimize breakage across updates. Direct changes to the images will be replaced by new updated images. Overwriting a large updated script with an old fixed version is likely to introduce subtle errors. Thus one viable method is to choose a small script that is unlikely to change and inject our code in it. One current example is build/examples/platform.sh.

We can always gain SSH access in this way because the application filesystem images resides on the SD card and we have physical access to it.

After getting SSH access, `sudo -i` to become root because user "st" is a sudoer.

## Exploring driving protocol

Visual inspection shows Beam's main computer connects to the motor board at the bottom through a USB serial port. This corresponds to the device `/dev/ttyACM0`.

This shows texclient (pid 1653) is communicating with the motor board using this device as file descriptor 33.
```
Brown University @0 stable_2.10.4 st@beam101095228:~$ sudo lsof /dev/ttyACM0
COMMAND    PID USER   FD   TYPE DEVICE SIZE/OFF  NODE NAME
texclient 1653   st   33uW  CHR  166,0      0t0 29512 /dev/ttyACM0
```

Use strace to monitor what is happening over this device:
```
Brown University @0 stable_2.10.4 st@beam101095228:~$ sudo strace -etrace=read,write -f -p1653 2>&1 | grep --line-buffered '(33,'
...
[pid  1751] write(33, "\252\252UU\1\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"..., 48) = 48
...
[pid  1751] read(33, "33\314\314\3\0\1\0\0\0\0\0\0\0\0\0q\1\0\0w\2\0\0,\0\0\0\253\267\f\0"..., 4096) = 128
```
