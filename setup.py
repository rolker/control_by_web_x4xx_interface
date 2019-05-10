from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['cbw_X4xx_interface'],
    package_dir={'': 'src'},
)

setup(**d)
