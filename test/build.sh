#!/bin/bash

CONFIG=Debug
GENERATOR="Unix Makefiles"

cmake . -Bbuild -DCMAKE_BUILD_TYPE=$CONFIG -G "$GENERATOR"
cmake --build build --config $CONFIG