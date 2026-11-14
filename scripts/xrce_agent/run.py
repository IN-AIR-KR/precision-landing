"""Start the serial XRCE Agent with the real flight profile."""
import os
from pathlib import Path
import sys

import yaml


config = yaml.safe_load(Path(sys.argv[1]).read_text())['px4']
uart = config['uart']
os.environ['ROS_DOMAIN_ID'] = str(config.get('domain_id', 0))
agent = sys.argv[2]
os.execv(agent, [agent, 'serial', '-D', uart['device'], '-b', str(uart['baud'])])
