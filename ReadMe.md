# dSPACE Indy Autonomous Challenge Simulator
This repository contains the current state of the SUT-TE-bridge implementation for the dSPACE IAC simulation environment. The bridge enables the data exchange between the dSPACE ASM car and environment model and ROS2. If you encounter bugs or missing features, feel free to open an issue and/or create a branch for your implementation and merge it back into the main branch later on. Long term it is planned to make this repository open source, for the moment every team member needs to request access separately.

This documents enables you to get started with the minimal local setup of the dSPACE Indy Autonomous Challenge simulator.

## Prerequisites
You need to have the following tools installed:
- Docker (e.g. Docker Desktop)
- Docker Compose (already installed if you use Docker Desktop)

You need to have access to the following instances:
- dSPACE IAC license server
- SIMPHERA AWS docker registry

## How to
The following instructions assume an execution in a Linux environment, either on a Linux host system or in WSL. This means that all given commands and scripts are written for Linux. However the execution also works for Windows and Mac, you just need to adapt the commands and scripts slightly for your preferred OS.

### Execute
1. Start Docker Desktop
2. Navigate into the folder, where the Docker compose is located.
3. Open a terminal and execute `docker compose -f docker-compose_sut_te_bridge_foxglove.yml up`
4. Start Foxglove
    1. Open Foxglove for visualisation `https://simphera-iac.dspace-dev.com/foxglove/`
    2. Click on *Open connection* and connect to the default address *ws://localhost:8765*
    3. Load layout from ApplCSH\SW\Foxglove\iac-layout-basic.json. CLick View->Import layout from file->Select json file
5. Open a second terminal, attach to the running sut_te_bridge container, build the solution and start the execution. In our example that means the following steps:
    1. Switch the sut_te_bridge image to the dev version in the docker compose and mount the directories including the required sources into the container
        1. Image name: `722180079256.dkr.ecr.eu-central-1.amazonaws.com/dspace/iac_sut_te_bridge_dev`
        2. volumes:
            - ./dSPACE-IAC-sut-te-bridge/ros2_bridge_ws:/root/ros2_bridge_ws
            - ./dSPACE-IAC-sut-te-bridge/runtime_scripts:/root/runtime_scripts
    2. `docker exec -it sut_te_bridge bash`
    3. `./ros2build`
    4. `./ros2run`
6. Open a third terminal and start your stack. In our example that includes attaching to the driving_stack container and executing the start script:
    1. `docker exec -it driving_stack bash`
    2. `./ros2build`
    3. `./ros2run`
7. To shut down the simulation, open another terminal and execute `docker compose -f docker-compose_sut_te_bridge_foxglove.yml down --remove-orphans`

### Contribute
In general the bridge is maintained by dSPACE, so if you find any missing features or bugs it would be great if you make use of the github issue feature to share them with us.
If you would like to package the simulation data in ROS publishers/subscribers, that are currently not supported e.g. CAN or dbw_raptor messages, the following section should provide some guidance how to easily achieve that.
In case that these additional interfaces might be useful to other teams, it would be great, if you could push your changes to a seperate branch and create a pull request, so that they become available for the rest of the community.
1. Add declaration of the publishers/subscribers to the [bridge.h](ros2_bridge_ws/src/sut_te_bridge/include/bridge.h) header
2. Initialize publishers similar to lines 112-144 in [bridge.cpp](ros2_bridge_ws/src/sut_te_bridge/src/bridge.cpp#L112-L144)
3. Initialize subscribers similar to lines 147-148 in [bridge.cpp](ros2_bridge_ws/src/sut_te_bridge/src/bridge.cpp#L147-L148)
4. Implement publisher function which accesses the data stored in this->CanBus object, creates ROS messages from it and publishes it using your publisher from step 2
5. Add call to the publisher function to the [publishSimulationState function](ros2_bridge_ws/src/sut_te_bridge/src/bridge.cpp#L278-L323)
6. Implement callback functions for your subscribers from step 3, which read the corresponding ROS messages and write the data to this->feedbackCmd object
