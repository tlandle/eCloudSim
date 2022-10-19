FROM ubuntu:20.04

WORKDIR /root

RUN apt-get update && apt-get install -y software-properties-common

RUN	add-apt-repository ppa:deadsnakes/ppa && \
	apt-get install -y python3.7 && \
	apt-get install -y ffmpeg libsm6 libxext6 && \
	apt-get install -y python3-pip && \
	apt-get install -y python3.7-distutils && \
	apt-get install -y python3-apt

RUN python3.7 -m pip install --upgrade pip && python3.7 -m pip install --upgrade setuptools 

RUN python3.7 -m pip install grpcio && python3.7 -m pip install protobuf

COPY requirements.txt .

RUN python3.7 -m pip install -r requirements.txt

RUN python3.7 -m pip install carla==0.9.12

RUN python3.7 -m pip install coloredlogs

RUN python3.7 -m pip install grpcio-tools

COPY . .

EXPOSE 5555/tcp

# gRPC
EXPOSE 50051/tcp 

# Carla
EXPOSE 2000/tcp

RUN python3.7 -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/sim_api.proto

CMD python3.7 vehiclesim.py







