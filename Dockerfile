#FROM ubuntu:20.04
FROM nvidia/cuda:11.0.3-base-ubuntu20.04

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

RUN python3.7 -m pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116

# RUN python3.7 -m pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt

# replaces requirements.txt download
RUN python3.7 -m pip install gitpython>=3.1.30 \
                matplotlib>=3.3 \
                numpy>=1.18.5 \
                opencv-python>=4.1.1 \
                Pillow>=7.1.2 \
                psutil \
                PyYAML>=5.3.1 \
                requests>=2.23.0 \
                scipy>=1.4.1 \
                thop>=0.1.1 \
                torch>=1.7.0 \
                torchvision>=0.8.1 \
                tqdm>=4.64.0 \
                pandas>=1.1.4 \
                seaborn>=0.11.0 \
                setuptools>=65.5.1

RUN apt-get install -y libglfw3-dev

RUN python3.7 -m pip install open3d

COPY . .

EXPOSE 5555/tcp

# gRPC
EXPOSE 50051/tcp

# Carla
EXPOSE 2000/tcp


RUN python3.7 -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/sim_api.proto


ENTRYPOINT [ "python3.7", "vehiclesim.py" ]