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

where `RPD-BOOT` partition stores the bootstrap files, `RPD-STORE` stores application files, and `RPD-LOG` logs.

### RPD-BOOT

The boot partition contains the bootloader grub, the kernel `bzImage`, and the initramfs `main-0b182d3da87e-1425884538.474827051.sqsh`.

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

GRUB is configured to boot up the kernel with plain parameters:
```
menuentry 'RPD Default Image' --class os {
  strecordfail
  linux /bzImage loglevel=3 splash
}
```

The path of the initramfs seems hardcoded in the kernel. Its content can be extracted with squashfs-tools:
```
TODO
```
