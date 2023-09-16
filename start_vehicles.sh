#!/bin/bash

read -p "how many vehicle client containers do you want to start? " count
read -p "use ML (Y/n)? " use_ml
read -p "rebuild containers (Y/n)? " rebuild

if [[ "$rebuild" = "Y" || "$rebuild" = "y" ]]; then
    echo "rebuilding container image"
    sudo docker build -f Dockerfile -t vehicle-sim:latest .
fi

running_containers = $(docker ps | wc -l)
if (( running_containers > 1 )); then
    echo "stopping and removing all old containers..."
    sudo docker stop $(sudo docker ps -a -q)
    sudo docker rm $(sudo docker ps -a -q)
    sudo docker container ls
fi

echo "starting $count vehicle client vontainers..."

gpu = 0
num_gpus = $(nvidia-smi -L | wc -l)
for ((i=0; i<$count; i++))
do
    if [[ "$use_ml" = "Y" || "$use_ml" = "y" ]]; then
        sudo docker run --runtime=nvidia --gpus device=$gpu -d --network=host --name=container_$i -e "HOSTNAME=container_$i" -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY vehicle-sim --apply_ml
    else
        #sudo docker run --runtime=nvidia --gpus all -d --network=host --name=container_$i -e "HOSTNAME=container_$i" -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY vehicle-sim
        sudo docker run -d --network=host --name=container_$i -e "HOSTNAME=container_$i" vehicle-sim
    fi
    if ! (( gpu % num_gpus )); then
        gpu = 0
    else
        gpu = $((gpu++))
    fi
done

sudo docker container ls
