## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['vigir_trajectory_smoother'],
    scripts=['scripts/trajectory_plotter', 'scripts/trajectory_smoother'],
    package_dir={'': 'src'},
)

setup(**setup_args)
