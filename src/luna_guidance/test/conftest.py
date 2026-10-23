"""Lets `pytest` find the luna_guidance package without building the ROS workspace."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
