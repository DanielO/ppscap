#!/bin/sh
#
# Author: Daniel O'Connor <darius@dons.net.au>
#
#

# PROVIDE: ppscap
# REQUIRE: LOGIN
# KEYWORD: shutdown

# Add the following lines to /etc/rc.conf to enable ppscap:
# ppscap_enable="YES"
# ppscap_cmdflags="<set as needed>"

. /etc/rc.subr

name="ppscap"
rcvar=ppscap_enable

load_rc_config $name

: ${ppscap_enable="NO"}
: ${ppscap_user:=ntpd}

pidfile="/var/db/ntp/${name}.pid"
procname="/usr/local/bin/ppscap"
command="/usr/sbin/daemon"
command_args="-f -S -p ${pidfile} ${procname} ${ppscap_cmdflags}"

run_rc_command "$1"