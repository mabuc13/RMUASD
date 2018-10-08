from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/gcs_node.py','scripts/dockingstation.py','scripts/drone.py'],)
setup(**d)

