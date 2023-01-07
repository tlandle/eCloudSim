# OpenCDA + Distributed/Async Edge Computing in the Cloud

This software is an extension of the [OpenCDA simulation tool](https://github.com/ucla-mobility/OpenCDA). The following features are added as an extension of OpenCDA:

- Distrbuted/Asynchronous communication between OpenCDA(Edge)/Carla and Vehicle clients using gRPC
- Containerization of vehicle clients using Nvidia Docker 2 (supports local vehicle planning/perception)
- Plugable Algorithm for vehicle autonomous driving
- Support for Propagation Models
- Automation scripts for Cloud deployment of simulation using Ansible
- Metric/Evaluation gathering for simulation performance


## Installation

Install conda and opencda environment

https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html


Install Ortools

```
pip install --user ortools==9.3.10497 
```

Install k-means-constrained

```
pip install k-means-constrained==0.7.0
python -c "from k_means_constrained import KMeansConstrained"
```

Create gRPC stubs

```
python -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/sim_api.proto
```

For perception, install [Nvidia Docker 2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

## Usage

Activate the conda enviroment

```
conda activate opencda
```


Start the Carla server

```
./CarlaUE4.sh
./CarlaUE4.sh -RenderOffScreen # to run headless
```

Run opencda vehicle test

```
python opencda.py -t single_2lanefree_carla -v 0.9.12
python opencda.py -t multi_2lanefree_carla -v 0.9.12
python opencda.py -t ecloud_edge_scenario -v 0.9.12
```

Build Docker image for vehicle clients
```
sudo docker build -t vehicle-sim .
```

Run vehicle containers
```
sudo bash start_vehicles.sh
```

Stop and remove vehicle containers
```
sudo bash stop_vehicles.sh
```

Docs to run Simulation in the cloud using Ansible: [README](ansible/README.md)
