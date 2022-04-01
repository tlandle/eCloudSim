FROM ubuntu:18.04

WORKDIR /root

#RUN apt-get update && apt-get install -y python3 \
#	python3-pip

RUN apt-get update && apt-get install -y software-properties-common && \
	add-apt-repository ppa:deadsnakes/ppa && \
	apt-get install -y python3.7 && \
	apt-get install -y ffmpeg libsm6 libxext6 && \
	apt-get install -y python3-pip

RUN python3.7 -m pip install --upgrade setuptools && python3.7 -m pip install --upgrade pip

COPY . .

RUN python3.7 -m pip install -r requirements.txt

RUN python3.7 -m pip install carla==0.9.12

EXPOSE 5555/tcp

CMD python3.7 vehiclesim.py






