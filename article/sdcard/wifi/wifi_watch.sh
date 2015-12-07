#!/bin/sh

ckfile=/sys/class/net/wlan0/operstate

on_wifi_up() {
    echo Wifi is up
    sleep 5
    /tmp/fuse_d/wifi/sta.sh
}

# Wait up to 5 minutes for wifi to come up.
wait_wifi_up() {
    n=0
    grep up $ckfile
    rc=$?

    if [ $rc -eq 0 ]; then
        on_wifi_up
    fi

    while [ $n -ne 36 ] && [ $rc -ne 0 ]; do
        n=$(($n + 1))
        sleep 10
        grep up $ckfile
        rc=$?

        if [ $rc -eq 0 ]; then
            on_wifi_up
        fi
    done
}

wait_wifi_up

