#!/bin/sh
# Ubuntu 22.04 log4cxx headers need C++17. Several packages still pass -std=c++14.
exec /usr/bin/c++ "$@" -std=c++17
