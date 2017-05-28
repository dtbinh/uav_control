from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

sourcefiles = ['hw_interface.pyx']
ext_modules = [Extension("hw_interface",
                          sourcefiles,
                          language="c++"
                          )]

setup(
  name = 'i2c hw_interface',
  cmdclass = {'build_ext': build_ext},
  ext_modules = ext_modules
)
