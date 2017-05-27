import distutils.core import setup
from Cython.Build import cythonize
from Cython.Distutils import build_ext
from distutils.extension import Extension


setup(
        ext_modules = cythonize("test.pyx")
        )
#setup(
#        cmdclass = {'build_ext': build_ext},
#        ext_modules = [Extension("c_ctrl",['test.pyx'],language='c++')],
#        )
