#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ika_plate_rct_digital_driver'],
    package_dir={'': 'src'}
)

setup(**d)
