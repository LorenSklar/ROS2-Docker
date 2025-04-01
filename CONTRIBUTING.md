# Contributing to ROS2 Docker Development Environment

Thank you for your interest in contributing to this project! This document provides guidelines and steps for contributing.

## How to Contribute

1. Fork the repository
2. Create a new branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## Development Setup

1. Clone your fork:
```bash
git clone https://github.com/YOUR_USERNAME/ros2_docker.git
cd ros2_docker
```

2. Build the Docker image:
```bash
docker-compose build
```

3. Start the container:
```bash
docker-compose up -d
```

## Code Style

- Follow the ROS2 Python style guide for Python code
- Use meaningful commit messages
- Keep commits focused and atomic
- Update documentation as needed

## Testing

Before submitting a pull request:
1. Test your changes locally
2. Ensure the demo nodes work:
```bash
ros2 run demo_nodes_py talker
ros2 run demo_nodes_py listener
```

## Reporting Issues

When reporting issues, please include:
- Your operating system and version
- Docker version
- Steps to reproduce
- Expected behavior
- Actual behavior

## Questions?

Feel free to open an issue for any questions or concerns. 