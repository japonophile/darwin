#!/usr/bin/env python

import re
import sys

from setuptools import setup, find_packages


def version():
    with open('darwin_mini/_version.py') as f:
        return re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", f.read()).group(1)

extra = {}
if sys.version_info >= (3,):
    extra['use_2to3'] = True

setup(name='darwin-mini',
      version=version(),
      packages=find_packages(),

      install_requires=['pypot >= 3', 'hampy'],

      include_package_data=True,
      exclude_package_data={'': ['README', '.gitignore']},

      zip_safe=False,

      author='https://github.com/japonophile/darwin/graphs/contributors',
      author_email='antoine@japonophile.com',
      description='Darwin Mini Software Library',
      url='https://github.com/japonophile/darwin',
      license='MIT LICENSE',

      **extra
      )
