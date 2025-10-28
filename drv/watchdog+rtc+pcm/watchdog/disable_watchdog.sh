
echo 14 >  /sys/class/gpio/export 
echo out >  /sys/class/gpio/gpio14/direction 
#cat /sys/class/gpio/gpio14/direction 
echo 0 > /sys/class/gpio/gpio14/value
