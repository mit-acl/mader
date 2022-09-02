#!/bin/bash

# get NUC's number
num=$1
# get the interface name
interface=$(iw dev | awk '$1=="Interface"{print $2}')

# bring network manager down
sudo systemctl stop NetworkManager.service
# then set up ad-hoc
sudo ip link set $interface down
sudo iwconfig $interface mode ad-hoc
sudo iwconfig $interface channel auto
sudo iwconfig $interface essid “mesh”
sudo iwconfig $interface key 1234567890

# if sudo ip link set $interface up gives you an error
# sudo rfkill list all
# sudo rfkill unblock wifi

sudo ip link set $interface up

# add address to your self
sudo ip addr add 192.168.100.$num/24 dev $interface
# add gateway (which is yourself)
sudo route add default gw 192.168.100.$num $interface

