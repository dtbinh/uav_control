clean:
	rm -f -r build/
	rm -f *.so

clean-cpp:
	rm -f hw_interface.cpp

clean-all: clean clean-cpp

.PHONY: build
build: clean
	python setup.hw_interface.py build_ext --inplace

.PHONY: cython-build
cython-build: clean clean-cpp
	python setup.hw_interface.py build_ext --inplace --use-cython
