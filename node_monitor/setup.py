from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/node_monitor_node.py'],)
setup(**d)

