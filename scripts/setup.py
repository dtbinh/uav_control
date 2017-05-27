from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

sourcefiles = ['c_ctrl.pyx']
ext_modules = [Extension("c_ctrl",
                          sourcefiles,
                          language="c++"
                          )]

setup(
  name = 'test',
  cmdclass = {'build_ext': build_ext},
  ext_modules = ext_modules
)
