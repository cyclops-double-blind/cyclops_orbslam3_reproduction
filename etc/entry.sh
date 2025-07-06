#!/bin/bash
if [ -z "${USER_ID}" ]; then
    USER_ID=1000
fi

/etc/init.d/dbus start
/usr/sbin/avahi-daemon -D --no-chroot

grep devel /etc/passwd > /dev/null 2>&1
if [ ! $? -eq 0 ] ; then
    echo "creating new user 'devel' with UID: $USER_ID"
    useradd --shell /bin/bash -u $USER_ID -o -d /home/devel devel \
        && mkdir -p /home/devel \
        && chown -R devel:devel /home/devel
fi

exec gosu devel /etc/init.sh "$@"
