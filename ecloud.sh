#!/bin/bash

count=0
rebuild=0
use_ml=0
kill=0
environment="local"
clean=0

__usage="script usage: $(basename $0) [-n num_cars] [-r] [-p] [-k] [-c]

Options:
-n #: number of containers to run
-r: rebuild containers
-p: enable perception
-e env: environment to run - 'local' or 'azure' | default: 'local'
-k: kill existing containers and exit
-c: clean up on exit"
while getopts 'n:rpkc' OPTION; do
    case "$OPTION" in
        n)
            count="$OPTARG"
            echo "$OPTARG containers requested"
            ;;
        r)
            echo "container rebuild request"
            rebuild=1
            ;;
        p)
            echo "scenario will run with perception enabled"
            use_ml=1
            ;;
        l)
            environment="$OPTARG"
            echo "running in $OPTARG environment"
            ;;
        k)
            kill=1
            ;;
        c)
            echo "will cleanup after a successful scenario"
            clean=1
            ;;
        ?)
            echo "$__usage">&2
            exit 1
            ;;
    esac
done
shift "$(($OPTIND -1))"

if (( rebuild == 1 )); then
    echo "rebuilding container image"
    sudo docker build -f Dockerfile -t ecloud-client:latest .
    if (( count == 0 && kill == 0 )); then
        exit 1
    fi
fi

if (( kill == 1 )); then
    # if (( count > 0 || use_ml > 0 )); then
    #     echo "WARNING: -k flag should be run only with -r or be the only flag"
    # fi
    running_containers=$(docker ps -a -q | wc -l)
    if (( running_containers > 0 )); then 
        echo "stopping and removing all old containers..."
        sudo docker stop $(sudo docker ps -a -q)
        sudo docker rm $(sudo docker ps -a -q)
        sudo docker container ls
        exit 1
    else
        echo "no containers running"
        exit 1
    fi
fi

if (( count == 0 )); then
    echo "WARNING: number of containers required"
    echo "$__usage">&2
    exit 1
fi

running_containers=$(docker ps -a -q | wc -l)
if (( running_containers > 0 )); then
    echo "WARNING: you had old containers - prefer to run stop_vehicles before starting a new scenario." 
    echo "stopping and removing all old containers..."
    sudo docker stop $(sudo docker ps -a -q)
    sudo docker rm $(sudo docker ps -a -q)
    sudo docker container ls
fi

echo "starting $count vehicle client containers..."

gpu=0
num_gpus=0
if (( use_ml == 1 )); then
    num_gpus=$(nvidia-smi -L | wc -l)
    echo "this machine has $num_gpus gpu cores"
fi

for ((i=0; i<$count; i++))
do
    if (( use_ml == 1 )); then
        echo "container $i pinned to gpu $gpu"
        sudo docker run --runtime=nvidia --gpus device=$gpu -d --network=host --name=ecloud_client_$i -e "HOSTNAME=ecloud_client_$i" -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ecloud-client --apply_ml --environment $environment
        ((gpu++))
        #echo "$gpu % $num_gpus = $(( gpu % num_gpus ))"
        if (( $(( gpu % num_gpus )) == 0 )); then
            echo "resetting gpu round robin"
            gpu=0
        fi   
    else
        #sudo docker run --runtime=nvidia --gpus all -d --network=host --name=ecloud_client_$i -e "HOSTNAME=ecloud_client_$i" -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ecloud-client
        sudo docker run -d --network=host --name=ecloud_client_$i -e "HOSTNAME=ecloud_client_$i" ecloud-client --environment $environment
    fi
done

sudo docker container ls

loop=1
while ( (( loop == 1 )) );
do
    for ((i=0; i<$count; i++))
    do
        if test "$( docker logs ecloud_client_$i 2> >(grep -i "end received") | wc -l )" -gt "1"; then
            echo "OK: scenario has completed successfully"
            loop=0
            break
        fi
        if test "$( docker logs ecloud_client_$i 2> >(grep -i "error") | wc -l )" -gt "0"; then
            echo "ERROR: ecloud_client_$i has crashed based on error"
            docker logs ecloud_client_$i
            loop=2
            break
        fi
        if test "$( docker logs ecloud_client_$i 2> >(grep -i "traceback") | wc -l )" -gt "0"; then
            echo "ERROR: ecloud_client_$i has crashed based on traceback"
            docker logs ecloud_client_$i
            loop=2
            break
        fi
        if test "$( docker logs ecloud_client_$i 2> >(grep -i "unknown") | wc -l )" -gt "0"; then
            echo "ERROR: ecloud_client_$i has a gRPC error"
            docker logs ecloud_client_$i
            loop=2
            break
        fi
    done
    if ( (( loop == 1 )) ); then
        echo -n -e 'working.\r'
        sleep 1
        echo -n -e 'working..\r'
        sleep 1
        echo -n -e 'working...\r'
        sleep 1
        echo -n -e '             \r'
    fi
done

if (( loop == 0 && clean == 1 )); then
    echo "stopping and removing all old containers..."
    sudo docker stop $(sudo docker ps -a -q)
    sudo docker rm $(sudo docker ps -a -q)
    sudo docker container ls
fi