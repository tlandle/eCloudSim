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

- Edge WP push fixup

  - sim API pushes WP buffer for all vehicles

  - gRPC server sends down WP buffer for specific vehicle

- random seeding in config

- if start is random, destination should also be random

  - hardcode minimum distance; TODO configurable, but this likely doesn't add much

- `ecloud` sections in YAML

- Back Off Config to YAML for Sleep Timing

    - Base initial wait off last world_tick_time; server to send

- Continue To Apply Control To Keep Vehicles From Driving Off Once Complete

- Test Different Backoffs

- Test Perf of Multiple gRPC Servers

    - If benefit, YAML to specify number of servers; % vehicle id to specify which server # to connect to

- YAML fixup to allow randomized spawning or explicit

- YAML to allow randomized destination

- YAML to specify num cars (with randomized locations & destination - or single destination)

Top Level

```yaml
# eCloud perception
define: &perception_is_active false
...
# eCloud
ecloud:
  perception_active: *perception_is_active
  distributed: true # set to false or comment out to run "standard" OpenCDA sequential sims
  num_servers: 2 # % num_cars to choose which port to connect to. 2nd - nth server port: p = 50053 + ( n - 1 )
  server_ping_time_s: 0.005 # 5ms
  client_world_time_factor: 0.9 # what percentage of last world time to wait initially
  client_ping_spawn_s: 0.05 # sleep to wait between pings after spawn
  client_ping_tick_min_s: 0.01 # minimum sleep to wait between pings after spawn
  client_ping_tick_backoff: 0.5 # how much to drop sleep each successive ping
```

Scenario

```yaml
# define scenario.
scenario:
  ecloud: 
    num_cars: 128
    spawn_type: random # random || explicit
    destination_type: explicit # random || explicit
    done_behavior: destroy # destroy || control
  single_cav_list: 
    - <<: *vehicle_base
      destination: [606.87, 145.39, 0]
      behavior: # overrides
        <<: *base_behavior
        max_speed: 100 # maximum speed, km/h
        tailgate_speed: 111
        overtake_allowed: false
        local_planner:
          <<: *base_local_planner
          debug_trajectory: true
          debug: true
```