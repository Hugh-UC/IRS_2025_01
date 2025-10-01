from setuptools import find_packages
from setuptools import setup

setup(
    name='tm_test',
    version='1.0.2',
    packages=find_packages(
        include=('tm_test', 'tm_test.*')),
)
