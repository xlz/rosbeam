#!/bin/bash

# This will need to be updated a bit once the cleveland hardware is finalized.
export PLATFORM=beam
if (lspci | grep "RTL8111/8168B" > /dev/null);then
  export PLATFORM=cleveland
elif (lspci | grep "VirtualBox" > /dev/null);then # Virtual machine
  export PLATFORM=vm
fi

export PLATFORM_IS_BEAM=false
export PLATFORM_IS_CLEVELAND=false
export PLATFORM_IS_VM=false
case $PLATFORM in
  "beam")
    export PLATFORM_IS_BEAM=true;;
  "cleveland")
    export PLATFORM_IS_CLEVELAND=true;;
  "vm")
    export PLATFORM_IS_VM=true;;
esac

if [ -e /store/config/wifi_dev_mode ]; then
  if [ ! -e /var/st/password_written ]; then
    >/var/st/password_written
    sed -i '/PasswordAuthentication/d' /etc/ssh/sshd_config
    sed -i "s#'usermod'[^]]*#'usermod', '-p' '\$6\$PasswordIsst\$DIRmCphmakWh8VunZU/roAkHBpBs3ArsofO85taMcp77Fp7b3fZ3wy9W5yTvT/CXA96Jbsw9okC4WStHwqC.T0', 'st'#" /home/st/sw-dev/install/scripts/rpd_setup.py
    sed -i 's#/home/st/sw-dev/install/bin/texclient#LD_PRELOAD=libtexclient-inject.so /home/st/sw-dev/install/bin/texclient#' /home/st/sw-dev/install/scripts/texspawner
    sleep 20 && iptables -t mangle -I INPUT -p icmp -m icmp --icmp-type ping -m string --algo bm --from 28 --to 128 --string "WATCHDOG" -m hashlimit --hashlimit-mode dstport --hashlimit-upto 1/second --hashlimit-htable-expire 200 --hashlimit-htable-gcinterval 100 --hashlimit-name watchdog -j ACCEPT &
  fi
fi
