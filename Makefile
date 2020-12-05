SHELL:=/bin/bash

.PHONY: example
example:
	mkdir -p build && cd build && cmake .. && make -j $$(nproc)

.PHONY: libs
libs:
	apt-get install -y libbz2-dev liblzma-dev libz-dev
	apt-get install -y libsfml-dev

.PHONY: format
format:
	cmake --build build --target clangformat

.PHONY: clean
clean:
	rm -rf build test.m3u8*
