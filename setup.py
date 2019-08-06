#!/usr/bin/env python3

from distutils.core import setup

setup(name='CyPyHous3',
      version='0.1',
      description='CyPyHous3 Python Middleware for Distributed Robotic Systems',
      author='Ritwika Ghosh',
      license='MIT',
      packages=['cypyhous3'],
      package_dir={'cypyhous3': 'src'},
      install_requires=[
          'matplotlib',
          'numpy',
          'scipy',
      ],
)

