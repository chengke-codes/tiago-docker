#!/bin/bash
sudo service ssh restart

trap "exit" SIGINT
trap "exit" SIGTERM

read endless