#!/bin/sh -e
if [ "`id -u`" != 0 ]; then
	echo requires root >&2
	exit 1
fi
st001_dev=`readlink -f /dev/st001`
texclient_pid=`pgrep texclient`
fd=`cd /proc/$texclient_pid/fd && find -L * -samefile $st001_dev 2>/dev/null`
if [ ! "$fd" ]; then
	echo st001 fd not found >&2
	exit 1
fi
/store/dev/strace -f -p$texclient_pid -q -etrace=write -esignal= -s0 2>&1 | grep -m1 "write($fd,"
