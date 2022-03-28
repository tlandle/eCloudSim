# Design Description for Deployment of Multi-vehicle Simulation

![Design document](https://github.com/tlandle/OpenCDA/blob/dist/deployment/deployment.png)

## Virtual Machines

The deployment of a multi-vehicle simulation will involve several VM types:
1. Carla server - will need to be installed on a virtual machine with a GPU. This VM is running the Carla server and doing simulation rendering.
2. OpenCDA - the OpenCDA framework needs to be installed on a VM, which is the master for the simulation. This VM is responsible for initiating the simulation, connecting to each simulation vehicle and passing simulation parameters, and "tick" the simulation to cause it to advance a single time step.
3. Worker nodes - we will use multiple worker nodes that can host one or more Docker containers for the simulated vehicle. Since we simulated vehicle can use a neural network for doing perception of other vehicles, the worker nodes will need to be built on a GPU VM.

We will manually create a VM for the Carla server and OpenCDA. We will create an image of a worker node to encapsulate our desired worker node installation.

We will use docker_machine to create new workers from our image and to deploy our Docker container image to the worker, as well as starting/stopping the containers when we want to run a simulation. 
