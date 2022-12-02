#!/bin/bash

python3.7 -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/sim_api.proto

bash generate_map.sh

python3.7 vehiclesim.py
