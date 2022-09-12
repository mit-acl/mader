#!/bin/bash

sudo ip route add 192.168.15.0/24 via 192.168.100.201  #NUC1
sudo ip route add 192.168.16.0/24 via 192.168.100.202  #NUC2
sudo ip route add 192.168.17.0/24 via 192.168.100.203  #NUC3
sudo ip route add 192.168.18.0/24 via 192.168.100.204  #NUC4
sudo ip route add 192.168.19.0/24 via 192.168.100.205  #NUC5
sudo ip route add 192.168.20.0/24 via 192.168.100.206  #NUC6
sudo ip route add 192.168.21.0/24 via 192.168.100.207  #NUC7
sudo ip route add 192.168.22.0/24 via 192.168.100.208  #NUC8
sudo ip route add 192.168.23.0/24 via 192.168.100.209  #NUC9
sudo ip route add 192.168.24.0/24 via 192.168.100.210  #NUC10
sudo ip route add 192.168.25.0/24 via 192.168.100.211  #NUC11


sudo ip route add 192.168.100.1 via 192.168.100.201
sudo ip route add 192.168.100.2 via 192.168.100.202
sudo ip route add 192.168.100.3 via 192.168.100.203
sudo ip route add 192.168.100.4 via 192.168.100.204
sudo ip route add 192.168.100.5 via 192.168.100.205
sudo ip route add 192.168.100.6 via 192.168.100.206
sudo ip route add 192.168.100.7 via 192.168.100.207
sudo ip route add 192.168.100.8 via 192.168.100.208
sudo ip route add 192.168.100.9 via 192.168.100.209
sudo ip route add 192.168.100.10 via 192.168.100.210
sudo ip route add 192.168.100.11 via 192.168.100.211
