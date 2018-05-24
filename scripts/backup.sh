#!/bin/bash

usage ()
{
  echo 'usage   : ./backup.sh robot_address robot_id'
  echo 'example : ./backup.sh ubuntu@192.168.0.213 3'
  echo '          ./backup.sh bot3-wlan 3'
  exit 1
}

if [ "$#" -ne 2 ]
then
  usage
fi

mkdir -p $HOME/dconfig
scp -r $1:/home/ubuntu/humanoid/src/robot/dconfig/$2 $HOME/dconfig/
