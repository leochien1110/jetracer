#!/usr/bin/env python3
"""Version information for jetracer package."""

__version__ = "0.1.0"
__version_info__ = tuple(int(num) for num in __version__.split('.'))

# Release notes for this version
__release_notes__ = """
JetRacer v0.1.0
===============

Features:
- Automatic ESC direction change handling with 0.5s timing
- Debug logging system with configurable output levels
- Thread-safe operation with proper state management
- Improved calibration utilities
- ROS2-ready package structure

Breaking Changes:
- debug_enabled now defaults to False for cleaner operation

Bug Fixes:
- Fixed ESC double-click reverse sequence timing
- Improved thread safety for concurrent operations
""" 