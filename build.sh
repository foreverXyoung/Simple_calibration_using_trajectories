#!/bin/bash
rm -r ./build ./bin
mkdir build && cd build
cmake .. && make -j24
