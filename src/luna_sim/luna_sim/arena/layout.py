"""
Helper file that reads an arena .json file. All needs needing to read such files should use this to do so.

Use:
  from luna_sim.arena.layout import load_layout

  layout = load_layout(self.declare_parameter('arena_json', '').value)
  start = layout['start_pose'] 
"""
import json


def load_layout(path):
    with open(path) as f:
        return json.load(f)