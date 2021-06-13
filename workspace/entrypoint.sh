#!/bin/bash
sudo systemctl restart ssh

trap "exit" SIGINT
trap "exit" SIGTERM

read endless