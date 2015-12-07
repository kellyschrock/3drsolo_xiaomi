sleep 10

t pwm 1 enable
sleep 0.75
t pwm 1 disable

lu_util exec '/tmp/fuse_d/wifi/wifi_watch.sh &'

t pwm 1 enable
sleep 0.75
t pwm 1 disable

# Uncomment following line for network keepalive watchdog; EDIT watchdog.sh BEFORE USING! 
# lu_util exec '/tmp/fuse_d/wifi/watchdog.sh'

