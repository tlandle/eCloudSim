#!/bin/bash

read -p "How many vehicle client containers do you want to start? " count

echo "Starting $count Vehicle Client Containers..."

for ((i=0; i<$count; i++))
do
    sudo docker run -d --network=host vehicle-sim
done

sudo docker container ls