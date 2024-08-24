from setuptools import find_packages
from setuptools import setup

setup(
    name='custom_interface_yolo',
    version='0.0.0',
    packages=find_packages(
        include=('custom_interface_yolo', 'custom_interface_yolo.*')),
)
