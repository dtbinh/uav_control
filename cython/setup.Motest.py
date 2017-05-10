from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

sourcefiles = ['MoTest.pyx']
ext_modules = [Extension("MoTest",
                          sourcefiles,
                          language="c++"
                          )]

setup(
  name = 'MotorTest app',
  cmdclass = {'build_ext': build_ext},
  ext_modules = ext_modules
)
