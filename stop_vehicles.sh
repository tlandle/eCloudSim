#!/bin/bash

echo "Stopping and Removing all old containers..."

sudo docker stop $(sudo docker ps -a -q)
sudo docker rm $(sudo docker ps -a -q)

sudo docker container ls