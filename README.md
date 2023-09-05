# OpenCDA + Distributed/Async Edge Computing in the Cloud

This software is an extension of the [OpenCDA simulation tool](https://github.com/ucla-mobility/OpenCDA). The following features are added as an extension of OpenCDA:

- Distrbuted/Asynchronous communication between OpenCDA(Edge)/Carla and Vehicle clients using gRPC
- Containerization of vehicle clients using Nvidia Docker 2 (supports local vehicle planning/perception)
- Plugable Algorithm for vehicle autonomous driving
- Support for Propagation Models
- Automation scripts for Cloud deployment of simulation using Ansible
- Metric/Evaluation gathering for simulation performance


## Installation

Install Carla and OpenCDA:

https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html


Install Ortools

```bash
pip install --user ortools==9.3.10497 
```

Install k-means-constrained

```bash
pip install k-means-constrained==0.7.0
python -c "from k_means_constrained import KMeansConstrained"
```

Create gRPC stubs

```bash
python -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/ecloud.proto
```

For perception, install [Nvidia Docker 2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

## Usage

Activate the conda enviroment

```bash
conda activate opencda
```

Start the Carla server

```bash
./CarlaUE4.sh
./CarlaUE4.sh -RenderOffScreen # to run headless
```

Run opencda vehicle test

```bash
python opencda.py -t single_2lanefree_carla -v 0.9.12
python opencda.py -t multi_2lanefree_carla -v 0.9.12
python opencda.py -t ecloud_edge_scenario -v 0.9.12
```

Build Docker image for vehicle clients
```bash
sudo docker build -t vehicle-sim .
```

Run vehicle containers
```bash
sudo bash start_vehicles.sh
```

Stop and remove vehicle containers
```bash
sudo bash stop_vehicles.sh
```

Docs to run Simulation in the cloud using Ansible: [README](ansible/README.md)

Scratchpad

```bash
# number of running Docker containers (+1)
docker ps -a | wc -l

# dump *all* logs
docker ps -q | xargs -L 1 docker logs

# create symlink to file
ln -s <source> <destination>
```

## ToDo List

- Back Off Config to YAML for Sleep Timing

- Continue To Apply Control To Keep Vehicles From Driving Off Once Complete

- Test Different Backoffs

- Test Perf of Multiple gRPC Servers

    - If benefit, YAML to specify number of servers; % vehicle id to specify which server # to connect to

- YAML fixup to allow randomized spawning or explicit

- YAML to allow randomized destination

- YAML to specify num cars (with randomized locations & destination - or single destination)