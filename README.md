# drone_swarm — 6-DOF Quadrotor Swarm Formation Control


This repository contains our ROS 2 Jazzy and Gazebo Harmonic simulation for **EN.605.613 (Introduction to Robotics)**.


To ensure everyone on the team has the exact same ROS 2 Jazzy + Gazebo Harmonic setup regardless of host operating system (**Windows, macOS, or Linux**), the entire environment is containerized with Docker and includes a browser-based VNC Ubuntu desktop for running GUI tools like **RViz2** and **Gazebo**.


---


## 1. Prerequisites

* **[Git](https://git-scm.com/downloads)** installed on your host machine.
* **[Docker Desktop](https://docs.docker.com/get-docker/)** (or Docker Engine + Docker Compose on Linux) installed and running.
Recommended Docker Desktop resources: At least **8 GB RAM** and **4 CPU cores** allocated in Docker settings.

---


## 2. First-Time Setup


Open a terminal on your computer (**PowerShell** on Windows, **Terminal** on macOS/Linux) and run:


### Step 1: Clone the Repository
```bash
git clone https://github.com/Mooseburger1/drone_swarm.git/drone_swarm.git
cd drone_swarm
```


### Step 2: Build and Start the Docker Container
```bash
docker compose up -d --build
```

 **Note:** The very first build will take 3–10 minutes to download the base ROS 2 Jazzy VNC image and install Gazebo Harmonic and ROS 2 packages. Subsequent starts (docker compose up -d) take only a few seconds.
### Step 3: Access the Development Environment
Once the container is running, you can interact with it in two ways:


1. **Browser VNC Desktop (for RViz2, Gazebo, and GUI apps):**
Open your web browser and go to: **[http://127.0.0.1:6080/](http://127.0.0.1:6080/)**
You will see a full Ubuntu desktop running inside the container. You can open terminals (LXTerminal) directly inside this desktop to launch GUI tools like rviz2 or gz sim.
2. **CLI Terminal from Your Host Machine:**
To open a bash shell inside the container (lands directly in /ros2_ws):
     
```bash
     docker compose exec drone_swarm bash
```
     


---


## 3. First Build of the ROS 2 Workspace


Inside a container terminal (at /ros2_ws), build the ROS 2 package with --symlink-install:
```bash
cd /ros2_ws
colcon build --symlink-install
source ~/.bashrc
```


---


## 4. Daily Developer Workflow (How to Edit & Sync Code)


The root of this drone_swarm repository on your computer is **live-mounted** to /ros2_ws inside the Docker container (volumes: - .:/ros2_ws).


### Editing Code
You can edit files using your favorite IDE on your host computer (e.g., **VS Code**, **PyCharm**) **or** inside the container.
Any file saved on your host machine updates **instantly** inside /ros2_ws in the container.
**You do NOT need to rebuild the Docker image when code changes.**

### Do I Need to Re-run colcon build After Editing a File?
Because we build with colcon build --symlink-install:
**Editing existing .py, .urdf, .sdf, or .launch.py files:** **No rebuild needed!** Just stop (Ctrl+C) and re-run your ros2 launch or ros2 run command.
**Adding a brand-new file, changing setup.py entrypoints, or changing package.xml:** Run:
 
```bash
  cd /ros2_ws
  colcon build --symlink-install
  source ~/.bashrc
```
 


### Pulling & Pushing Changes with GitHub
Run standard Git commands from your host terminal (the build/, install/, and log/ folders are automatically ignored by .gitignore):

```bash
# Pull latest changes from teammates
git pull
```


# Stage, commit, and push your changes
```bash
git add .
git commit -m "Describe your change"
git push
```

(After running git pull, if a teammate added new files or entrypoints, run

```bash
colcon build --symlink-install && source ~/.bashrc
```
inside the container.)


---


## 5. Running & Testing in RViz2


### Test the Single Quadrotor URDF (urdf_test.launch.py)
In a container terminal start the URDF state publishers:
   
```bash
ros2 launch drone_swarm urdf_test.launch.py
```

2. Open the VNC Desktop at **[http://127.0.0.1:6080/](http://127.0.0.1:6080/)**, open a terminal inside the VNC desktop, and start RViz2:
   
```bash
   rviz2
```

NOTE: I've also set env variables in the Dockerfile that should allow you to just run `rviz2` from another terminal inside the container and use your hose X11 renderer. This way you don't need to view from the VM in the browser
   
3. Inside **RViz2**:
* Under **Global Options -> Fixed Frame**, change map to base_link.
* Click **Add** (bottom-left) -> select **RobotModel** -> click **OK**.
* Expand **RobotModel** and set **Description Topic** to /robot_description.
---


## 6. Useful Docker Commands


| Task | Command |
| :--- | :--- |
| Start container (without rebuilding image) | docker compose up -d |
| Open a new terminal inside the container | docker compose exec drone_swarm bash |
| Stop the container | docker compose down |
| Rebuild image (only if Dockerfile changed) | docker compose up -d --build |
| View container startup logs | docker compose logs -f |