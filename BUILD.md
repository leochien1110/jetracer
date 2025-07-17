# Building JetRacer Python Package

This guide explains how to build and distribute the JetRacer package as a Python wheel for easy installation and use in projects like ROS2.

## 🏗️ Building the Package

### Prerequisites

```bash
# Install build tools
pip install build wheel setuptools twine

# Or install all dev dependencies
pip install -e ".[dev]"
```

### Build the Wheel

```bash
# Clean previous builds
rm -rf build/ dist/ *.egg-info/

# Build the package
python -m build

# This creates:
# - dist/jetracer-0.1.0-py3-none-any.whl (wheel)
# - dist/jetracer-0.1.0.tar.gz (source distribution)
```

### Verify the Build

```bash
# Check wheel contents
python -m zipfile -l dist/jetracer-0.1.0-py3-none-any.whl

# Check package metadata
python -m twine check dist/*
```

## 📦 Installation Options

### 1. Basic Installation (Core Features Only)

```bash
# Install from wheel
pip install dist/jetracer-0.1.0-py3-none-any.whl

# Or install from source
pip install .
```

### 2. Installation with AI Features

```bash
# Install with AI dependencies (PyTorch, OpenCV, etc.)
pip install ".[ai]"

# Or from wheel with extras
pip install "dist/jetracer-0.1.0-py3-none-any.whl[ai]"
```

### 3. Installation for Jupyter Development

```bash
# Install with Jupyter notebook support
pip install ".[jupyter]"
```

### 4. Complete Installation

```bash
# Install everything
pip install ".[all]"
```

## 🤖 ROS2 Integration

### Using in ROS2 Workspace

```bash
# Option 1: Install system-wide
sudo pip install dist/jetracer-0.1.0-py3-none-any.whl

# Option 2: Install in ROS2 workspace
cd ~/ros2_ws/src
pip install --user /path/to/jetracer/dist/jetracer-0.1.0-py3-none-any.whl

# Option 3: Install in development mode
cd ~/ros2_ws/src
git clone <jetracer-repo>
pip install -e jetracer/
```

### ROS2 Node Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from jetracer import NvidiaRacecar

class JetRacerNode(Node):
    def __init__(self):
        super().__init__('jetracer_node')
        self.car = NvidiaRacecar()
        self.car.disable_debug()  # Quiet operation for ROS2
        
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        # Convert ROS2 Twist to jetracer commands
        self.car.throttle = msg.linear.x
        self.car.steering = msg.angular.z

def main():
    rclpy.init()
    node = JetRacerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🚀 Distribution

### Local Distribution

```bash
# Copy wheel to target systems
scp dist/jetracer-0.1.0-py3-none-any.whl user@jetson:/tmp/
ssh user@jetson "pip install /tmp/jetracer-0.1.0-py3-none-any.whl"
```

### Private PyPI Server

```bash
# Upload to private PyPI
python -m twine upload --repository-url http://your-pypi-server dist/*
```

### GitHub Releases

```bash
# Create release and upload artifacts
gh release create v0.1.0 dist/jetracer-0.1.0-py3-none-any.whl dist/jetracer-0.1.0.tar.gz
```

## 🔧 Command Line Tools

After installation, you get command-line tools:

```bash
# Calibrate steering and throttle
jetracer-calibrate --steering --throttle --debug

# Dry run (no motor movement)
jetracer-calibrate --steering --dry-run
```

## 🛠️ Development Workflow

### Setup Development Environment

```bash
# Clone and install in development mode
git clone <repo>
cd jetracer
pip install -e ".[dev]"

# Run tests
pytest

# Format code
black jetracer/
isort jetracer/

# Type checking
mypy jetracer/
```

### Version Management

```bash
# Update version in jetracer/_version.py
echo '__version__ = "0.1.1"' > jetracer/_version.py

# Build new version
python -m build
```

## 📋 Dependencies Summary

- **Core**: `traitlets`, `adafruit-circuitpython-servokit`
- **AI**: `torch`, `torchvision`, `opencv-python`, `pillow`, `numpy`
- **Jupyter**: `jupyter`, `ipywidgets`, `matplotlib`
- **ROS2**: `rclpy` (managed by ROS2 environment)
- **Development**: `pytest`, `black`, `flake8`, `mypy`

## 🎯 Use Cases

1. **Standalone robotics projects**: Install core package
2. **AI/Computer Vision**: Install with `[ai]` extras
3. **ROS2 nodes**: Install system-wide or in workspace
4. **Research/Education**: Install with `[jupyter]` extras
5. **Development**: Install with `[dev]` extras

## 🔍 Troubleshooting

### Common Issues

```bash
# Permission errors on Jetson
sudo pip install --break-system-packages dist/jetracer-0.1.0-py3-none-any.whl

# ROS2 environment conflicts
pip install --user dist/jetracer-0.1.0-py3-none-any.whl

# Missing hardware dependencies
sudo apt-get install python3-smbus i2c-tools
```

### Verify Installation

```python
import jetracer
print(f"JetRacer version: {jetracer.__version__}")

# Test hardware (on Jetson with hardware)
car = jetracer.NvidiaRacecar()
car.enable_debug()
car.throttle = 0.1  # Should show debug output
car.throttle = 0.0
``` 