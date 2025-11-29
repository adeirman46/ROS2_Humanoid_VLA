from setuptools import find_packages
from setuptools import setup

setup(
    name='g1_controller',
    version='0.1.0',
    packages=find_packages(
        include=('g1_controller', 'g1_controller.*')),
)
