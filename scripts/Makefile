PYTHON?=pypy
all: clean build test
build:
	${PYTHON} setup_cc.py build_ext -if
	${PYTHON} setup.py build_ext --inplace -f
clean:
	@rm -fr build
	@rm -rf *.cpp *.c *.so *.html
test:
	${PYTHON} -m pytest -v
cc_all: clean cc_build cc_test
cc_build:
	pypy -m cython setup_cc.py build_ext -if
cc_test:
	pypy -m pytest tests/test_cc.py -v -s
cython:
	pypy -m cython cycontrol.pyx --cplus -a
