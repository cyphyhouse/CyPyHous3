#!/usr/bin/env python3

from distutils.core import setup

setup(name='src',
      version='0.1',
      description='CyPyHous3 Python Middleware for Distributed Robotic Systems',
      author='Ritwika Ghosh',
      license='MIT',
      packages=['src'],
      install_requires=[
          'matplotlib',
          'numpy',
          'scipy',
      ],
)

