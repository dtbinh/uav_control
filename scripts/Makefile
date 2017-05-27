PYTHON?=python
all: clean build test
build:
	${PYTHON} setup.py build_ext --inplace -f
clean:
	@rm -fr build
test:
	pytest
