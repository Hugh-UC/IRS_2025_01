# IRS_2025_01

**Contains lab documetation and packages for Industrial Robots and Systems - [CollaborativeRoboticsLab](https://github.com/CollaborativeRoboticsLab/industrial-robots-and-systems-world)**

**Group ID:** 

    Group_01

**Student ID's:**

    Hugh:   u3276400
    Dylan:  u3284475
    Kinley: u3264610 

***

<br>

## Getting Started

### 1. Clone the Repositories

First, you need to clone both the main lab repository and then this repository. You should clone them into your user's home directory (`~`).


Clone the '[Collaborative Robotics Lab](https://github.com/CollaborativeRoboticsLab/industrial-robots-and-systems-world.git)' lab repository
```sh
git clone https://github.com/CollaborativeRoboticsLab/industrial-robots-and-systems-world.git ~/industrial-robots-and-systems-world
```

Clone [this repository](https://github.com/Hugh-UC/IRS_2025_01.git)
```sh
git clone https://github.com/Hugh-UC/IRS_2025_01.git ~/IRS_2025_01
```

<br>

**! Optional:** You can clone this repository with a custom Docker development shell, see [Lab Shell](https://github.com/Hugh-UC/IRS_2025_01/?tab=readme-ov-file#lab-shell--docker-for-industrial-robots-and-systems).

<br>

## Launching the Containers

### 1. Start the Workspace

Once the setup script is complete, you can launch the Docker containers from the `industrial-robots-and-systems-world directory`.

Enter the folder
```sh
cd ~/industrial-robots-and-systems-world
```

Pull the latest docker containers
```bash
docker compose pull
```

Allow permission for UI interfaces from docker containers
```bash
xhost +local:root
```

Start the docker containers
```bash
docker compose up
```

Optional Flags:
```bash
docker compose up -d --build
```
- `docker compose up -d`: Use the `-d` (detached) flag to run the containers in the background.
- `docker compose up --build`: Use the `--build` flag to force a rebuild of the Docker image from the Dockerfile.

<br>

## Stopping the Containers

### 1. Stop the Workspace

To stop the containers, run the following command from the `industrial-robots-and-systems-world` directory.

Enter the folder
```sh
cd ~/industrial-robots-and-systems-world
```

Stop the docker containers
```bash
docker compose stop
```

Remove docker containers
```bash
docker compose down
```

<br>

***

<br>

## Lab Shell | Docker for Industrial Robots and Systems

This provides a pre-configured Docker environment for developing ROS 2 projects.

To clone this repository with the Lab Shell Docker
```sh
git clone --recurse-submodules https://github.com/Hugh-UC/IRS_2025_01.git ~/IRS_2025_01
```

<br>

For seperate installation instructions visit [https://github.com/Hugh-UC/IRS_Lab_Shell](https://github.com/Hugh-UC/IRS_Lab_Shell)

***
