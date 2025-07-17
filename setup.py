#!/usr/bin/env python3

import os
from setuptools import setup, find_packages

# Read long description from README
def read_long_description():
    here = os.path.abspath(os.path.dirname(__file__))
    try:
        with open(os.path.join(here, 'README.md'), 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        return "An easy to use AI racecar powered by NVIDIA Jetson Nano"

# Read version from version file
def read_version():
    here = os.path.abspath(os.path.dirname(__file__))
    version_file = os.path.join(here, 'jetracer', '_version.py')
    try:
        with open(version_file, 'r') as f:
            exec(f.read())
            return locals()['__version__']
    except FileNotFoundError:
        return "0.1.0"

# Core dependencies required for basic functionality
install_requires = [
    'traitlets>=5.0.0',
    'adafruit-circuitpython-servokit>=1.3.0',
]

# Optional dependencies for different use cases
extras_require = {
    'ai': [
        'torch>=1.9.0',
        'torchvision>=0.10.0',
        'opencv-python>=4.5.0',
        'pillow>=8.0.0',
        'numpy>=1.19.0',
    ],
    'jupyter': [
        'jupyter>=1.0.0',
        'ipywidgets>=7.6.0',
        'matplotlib>=3.3.0',
    ],
    'ros2': [
        # ROS2 dependencies will be managed by ROS2 package system
        'rclpy',  # This will be satisfied by ROS2 environment
    ],
    'dev': [
        'pytest>=6.0.0',
        'black>=21.0.0',
        'flake8>=3.9.0',
        'mypy>=0.910',
    ],
}

# Convenience meta-packages
extras_require['all'] = list(set(
    extras_require['ai'] + 
    extras_require['jupyter'] + 
    extras_require['dev']
))

setup(
    name='jetracer',
    version=read_version(),
    author='NVIDIA Corporation',
    author_email='jetracer@nvidia.com',
    description='An easy to use AI racecar powered by NVIDIA Jetson Nano',
    long_description=read_long_description(),
    long_description_content_type='text/markdown',
    url='https://github.com/NVIDIA-AI-IOT/jetracer',
    project_urls={
        'Documentation': 'https://github.com/NVIDIA-AI-IOT/jetracer/tree/master/docs',
        'Source': 'https://github.com/NVIDIA-AI-IOT/jetracer',
        'Tracker': 'https://github.com/NVIDIA-AI-IOT/jetracer/issues',
    },
    
    # Package configuration
    packages=find_packages(exclude=['tests*', 'docs*']),
    include_package_data=True,
    zip_safe=False,
    
    # Dependencies
    python_requires='>=3.7',
    install_requires=install_requires,
    extras_require=extras_require,
    
    # Package metadata
    license='MIT',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
        'Operating System :: POSIX :: Linux',
    ],
    keywords='jetson nano, AI, autonomous vehicle, robotics, computer vision, machine learning',
    
    # Entry points for command-line scripts
    entry_points={
        'console_scripts': [
            'jetracer-calibrate=jetracer.scripts.calibrate:main',
        ],
    },
)
