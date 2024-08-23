from setuptools import find_packages
from setuptools import setup

setup(
    name='yellow_ball_finder',
    version='0.0.0',
    packages=find_packages(
        include=('yellow_ball_finder', 'yellow_ball_finder.*')),
)
