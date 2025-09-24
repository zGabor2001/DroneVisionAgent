### Fire Drone RL Sim — README (Jazzy + Harmonic)

This repo launches **ROS 2 Jazzy + PX4 SITL + Gazebo Harmonic**. Garden is not packaged for Ubuntu 24.04, so Harmonic is the only supported option here.

---

### 1) Prerequisites (host)

* **Ubuntu 24.04 (Noble)** host
* **Docker Engine**
* **NVIDIA GPU (optional)** if you want hardware-accelerated rendering

---

### 2) Build the Docker image

```bash
sudo docker build -t fire-drone:jazzy-px4 .
```

This includes:

* ROS 2 **Jazzy** base
* Gazebo (gz) **Harmonic**
* `ros_gz` bridge packages (`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`)
* GUI and headless rendering support

---

### 3) Run the container

Headless:

```bash
sudo docker run --rm -it \
  --network host \
  --shm-size=4g \
  -e GZ_GUI=0 \
  -e QT_QPA_PLATFORM=offscreen \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v $HOME/DroneFlightAgent/ws/src/drone_sim:/repo/ws/src/drone_sim \
  --name fire-drone-sim \
  fire-drone:jazzy-px4
```

GUI (software rendering):

```bash
xhost +local:root
sudo docker run --rm -it \
  --network host \
  --shm-size=2g \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v $HOME/DroneFlightAgent/ws/src/drone_sim:/repo/ws/src/drone_sim \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --name fire-drone-sim \
  fire-drone:jazzy-px4
```

GUI (NVIDIA accelerated):

```bash
xhost +local:root
sudo docker run --rm -it \
  --network host \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --shm-size=2g \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/dri:/dev/dri \
  -v $HOME/DroneFlightAgent/ws/src/drone_sim:/repo/ws/src/drone_sim \
  --name fire-drone-sim \
  fire-drone:jazzy-px4
```

---

### 4) Build inside the container

```bash
source /opt/ros/jazzy/setup.bash
cd /repo/ws
rm -rf build install log
colcon build --symlink-install --merge-install --packages-skip px4
source install/setup.bash
cd /repo/ws/src/px4
make px4_sitl_default -j$(nproc)
```

---

### 5) Launch the sim

```bash
ros2 launch drone_sim px4_gz_bringup.launch.py \
  headless:=true px4:=true \
  px4_sim_model:=x500_custom \
  px4_sys_autostart:=4001
```

* Starts **Gazebo Harmonic**
* Starts **PX4 SITL** with the bridge
* Spawns/binds model depending on your arguments

---

### 6) Debugging tips

```bash
docker exec -it fire-drone-sim bash
gz --version
gz service -l | grep /world
gz model --list
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /clock

#Connect on the px4 console with
cd /repo/ws/src/px4
python3 Tools/mavlink_shell.py udp:0.0.0.0:14540

```

If PX4 times out, make sure Gazebo is running and that your `GZ_PLUGIN_PATH` is exported correctly.

---

### 7) Docker cleanup

```bash
docker ps -a --size
docker system df -v
docker container prune -f
docker image prune -a -f
docker builder prune -a -f
docker volume prune -f     # only if you’re sure unused volumes can go
docker system df -v
```
