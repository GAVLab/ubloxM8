#!/usr/bin/env python

from distutils.core import setup
from distutils.extension import Extension

setup(name="ublox_m8",
  ext_modules=[
    Extension("ublox_m8", sources = ["src/ublox_m8_python.cpp",
                                     "src/ublox_m8.cpp"],
    libraries = ["boost_python-py27", "boost_thread", "serial"],
    include_dirs = ["include/"],
    extra_compile_args=['-std=c++11'])
  ])

