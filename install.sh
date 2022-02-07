#!/bin/bash

read -p "Are you sure to install (y/n)?" REPLY
if [[ $REPLY =~ ^[Yy]$ ]]
then
    cp -a install/skel/. tiago_home/

    mkdir tiago_home/.gazebo

    cp install/gazebo/gui.ini tiago_home/.gazebo

    TIAGO_USER=`whoami` TIAGO_UID=`id -u $TIAGO_USER` TIAGO_GID=`id -g $TIAGO_USER` docker-compose build --build-arg TIAGO_USER=`whoami` --no-cache workspace
fi