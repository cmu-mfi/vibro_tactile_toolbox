# Vibro-Tactile Toolbox Docker

This folder contains Docker-related files for vibro_tactile_toolbox.

## Prerequisites

Before using the Docker components, ensure you have Docker installed on your system. You can download and install Docker from [here](https://www.docker.com/get-started).

## Contents

- Dockerfile: Defines the specifications for building the Docker image.
- build_docker.sh: Bash script to build the Docker image.
- run_docker.sh: Bash script to run the Docker container using the ROS_entrypoint.sh included in the image.
- new_terminal.sh: Bash script to open a new terminal using the already running image.

## Usage

### Building the Docker Image

To build the Docker image, run the following command:

```bash
./docker/build_docker.sh
```

### Running the Docker Container

To run the Docker container, run the following command:

```bash
./docker/run_docker.sh
```

Typically this first window should be used to run roscore

### Opening terminal windows within the running container

To open a new terminal inside of the already-running docker container, run the following command:

```bash
./docker/new_terminal.sh
```

To navigate to the ros/catkin workspace for the project use:

```bash
cd /vibro_tactile_toolbox/ros1_ws
```

It may be necessary to source the environment created by catkin to find the necessary packages:

```bash
cd /vibro_tactile_toolbox/ros1_ws
source build/setup.bash
source install/setup.bash
```
