#!/usr/bin/env python

from setuptools import setup

package_name = 'reference_system'

setup(
    name=package_name,
    version='1.0.0',
    description='Scripts and utilities for the evaluation of executor reference systems',
    license='Apache 2.0 License',
    author='Tobias Stark',
    author_email='tobias.stark@apex.ai',
    data_files=[
        f'share/{package_name}/package.xml',
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}'])],
    packages=[package_name],
    tests_require=[],
    entry_points={})
