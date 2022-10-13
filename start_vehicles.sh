#!/bin/bash

read -p "How many vehicle client containers do you want to start? " count

echo "Stopping and Removing all old containers..."

sudo docker stop $(sudo docker ps -a -q)
sudo docker rm $(sudo docker ps -a -q)


echo "Starting $count Vehicle Client Containers..."

for ((i=0; i<$count; i++))
do
    sudo docker run -d --network=host vehicle-sim
done

sudo docker container ls