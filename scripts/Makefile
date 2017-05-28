PYTHON?=python
all: clean build test
build:
	${PYTHON} setup_cc.py build_ext -if
	${PYTHON} setup.py build_ext --inplace -f
clean:
	@rm -fr build
test:
	pytest -v
cc_all: clean cc_build cc_test
cc_build:
	${PYTHON} setup_cc.py build_ext -if
cc_test:
	pytest tests/test_cc.py -v -s
