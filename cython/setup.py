from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy as np

setup(
  name = 'Demos',
  ext_modules=[
    Extension("hw_interface",
              # Note, you can link against a c++ library
              # instead of including the source
              sources=["i2c.pyx", "hw_interface.cpp"],
              language="c++"),
    ],
  cmdclass = {'build_ext': build_ext},
)
