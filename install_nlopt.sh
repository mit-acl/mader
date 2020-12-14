#!/bin/bash
# Author: Jesus Tordesillas Torres

#INSTALL NLOPT v2.6.2
##########################################
mkdir -p ~/installations/nlopt
cd ~/installations/nlopt
wget https://github.com/stevengj/nlopt/archive/v2.6.2.tar.gz
tar -zxvf v2.6.2.tar.gz 
cd nlopt-2.6.2/
cmake . && make && sudo make install