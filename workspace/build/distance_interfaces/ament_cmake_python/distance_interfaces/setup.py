from setuptools import find_packages
from setuptools import setup

setup(
    name='distance_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('distance_interfaces', 'distance_interfaces.*')),
)
