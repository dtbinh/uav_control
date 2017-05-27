from distutils.core import setup, Extension
from Cython.Distutils import build_ext
import numpy
module1 = Extension('cython_control',
                    extra_compile_args=['-std=c++11'],
                    include_dirs=['/usr/include/eigen3/Eigen','..',numpy.get_include()],
                    sources = ['cycontrol.pyx','src/controller.cpp'],
                    language = 'c++',
                    )

setup (name = 'cy_controller',
       version = '1.0',
       cmdclass = {'build_ext':build_ext},
       description = 'This is a demo package',
       ext_modules = [module1])
