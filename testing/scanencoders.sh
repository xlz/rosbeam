#!/bin/sh
# first, sudo visudo -f /etc/sudoers -> add /usr/bin/strace
# then, sudo sh ./getst001thread.sh
# then ssh -t st@172.18.167.180 /store/dev/st001jack/scanencoders.sh $threadid | python viz.py 
[ "$1" -a "$2" ] || exit 1
cd ${0%/*}
sudo strace -p $1 -etrace=read,write -qq -esignal= -x -s128 2>&1 | grep --line-buffered "($2, \"" | ./scanread
