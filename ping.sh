export MAC=$(cat /sys/class/net/eth0/address)

while true
do
  wget --post-data ${MAC} 192.168.1.100:12345 -T 2 -q -O/dev/null
  wget --post-data ${MAC} 192.168.1.110:12345 -T 2 -q -O/dev/null
  sleep 1
done
