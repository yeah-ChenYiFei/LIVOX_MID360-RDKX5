from setuptools import find_packages
from setuptools import setup

setup(
    name='fastlio_imu',
    version='0.0.0',
    packages=find_packages(
        include=('fastlio_imu', 'fastlio_imu.*')),
)
