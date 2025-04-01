# ROS2 Docker Development Environment

This repository contains a Docker-based development environment for ROS2 Humble, specifically configured for macOS with Apple Silicon (M1/M2) processors.

## Prerequisites

### Docker Desktop for Mac
1. Download Docker Desktop from [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Install Docker Desktop
3. Start Docker Desktop and wait for it to be running (you'll see the whale icon in your menu bar)

### Understanding ARM64 vs AMD64
- **ARM64 (aarch64)**: Used by Apple Silicon (M1/M2) Macs
- **AMD64 (x86_64)**: Used by Intel/AMD processors and Intel-based Macs

To check your Mac's architecture:
```bash
uname -m
```
- If it shows `arm64`, you have an Apple Silicon Mac
- If it shows `x86_64`, you have an Intel-based Mac

This matters because:
1. ROS2 packages need to match your system architecture
2. Running AMD64 containers on ARM64 requires emulation (slower)
3. Our Dockerfile is configured for ARM64 to ensure native performance

## Project Structure
```
ros2_docker_dev/
├── Dockerfile          # Container configuration
├── docker-compose.yml  # Container orchestration
├── .dockerignore      # Files to exclude from build
└── workspace/         # Your ROS2 workspace
    └── src/          # Source code directory
```

## Basic Setup

1. Clone this repository:
```bash
git clone <repository-url>
cd ros2_docker_dev
```

2. Build the Docker image:
```bash
docker-compose build
```

3. Start the container:
```bash
docker-compose up -d
```

4. Enter the container:
```bash
docker exec -it ros2_docker bash
```

## Testing Basic Setup

Inside the container, you can test ROS2 with the demo nodes:

1. Open two terminal windows
2. In the first terminal (talker):
```bash
ros2 run demo_nodes_py talker
```

3. In the second terminal (listener):
```bash
ros2 run demo_nodes_py listener
```

You should see messages being published and received between the nodes.

## GUI Support (Optional)

### Understanding GUI Support in Docker
Docker containers are isolated environments that don't have direct access to your system's display. To run GUI applications (like RViz2) from within a container, we use a web-based VNC solution (noVNC) that provides a virtual display and allows you to access it through your web browser.

This approach:
- Works reliably on all platforms, including Apple Silicon Macs
- Doesn't require XQuartz or other X11 forwarding setup
- Provides a secure, web-based interface to the container's display

### Setting Up GUI Support
1. Rebuild the container with GUI support:
   ```bash
   docker-compose down
   docker-compose build
   docker-compose up -d
   ```

2. Access the web interface:
   - Open http://localhost:6080/vnc.html in your web browser
   - Enter the VNC password (default is "password" unless you set VNC_PASSWORD)
   - Click "Connect"

3. Test the setup:
   ```bash
   docker exec -it ros2_docker bash
   rviz2
   ```

### Performance Considerations
- The virtual display uses software rendering, which can be CPU-intensive
- Default resolution is 1024x768 to balance performance and usability
- If you experience high CPU usage:
  - Close RViz2 when not in use
  - Use a lower resolution by modifying the Xvfb command in start.sh
  - Consider using headless mode for non-interactive tasks

### Customizing the Setup
You can customize the VNC password by setting the VNC_PASSWORD environment variable:
```bash
echo "VNC_PASSWORD=your_secure_password" > .env
docker-compose up -d
```

## Common Issues and Solutions

1. **Docker Desktop not running**
   - Ensure Docker Desktop is running (whale icon in menu bar)
   - Try restarting Docker Desktop

2. **Permission issues**
   - Docker Desktop needs full disk access on macOS
   - Check System Preferences → Security & Privacy → Privacy → Full Disk Access

3. **Network issues**
   - Ensure Docker Desktop has network access
   - Check Docker Desktop settings → Resources → Network

4. **GUI issues**
   - If noVNC page doesn't load:
     - Check if container is running: `docker ps`
     - Verify port 6080 is accessible: `curl localhost:6080`
     - Check container logs: `docker logs ros2_docker`
   - If VNC connection fails:
     - Verify VNC password is correct
     - Check if port 5901 is accessible
     - Try refreshing the browser page
   - If RViz2 is slow or unresponsive:
     - Reduce display resolution in start.sh
     - Close other resource-intensive applications
     - Consider using headless mode for non-interactive tasks

## Development Workflow

1. Your local `workspace` directory is mounted to `/root/ros2_ws` in the container
2. Create new ROS2 packages in `workspace/src`
3. Build packages using `colcon build` inside the container
4. Source your workspace: `source /root/ros2_ws/install/setup.bash`

## Stopping the Environment

1. Exit the container:
```bash
exit
```

2. Stop the container:
```bash
docker-compose down
```

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials/) 