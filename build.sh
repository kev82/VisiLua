#!/bin/sh

#build by default
if [ "$1" = "" ]; then
	cmd="build"
else
	cmd="$1"
fi

case "$cmd" in

#Dump the amalgamation to stdout
amalg) cat visilibity.hpp visilibity.cpp
	echo "static unsigned char vl_code[] = {"
	cat visilua.lua | luac -o - - | xxd -i
	echo "};"
	cat visilua.cc
	;;

#Compile the amalgamation
build) $0 amalg | \
 g++ -std=c++11 -fPIC -shared -O2 -x c++ -o visilua.so - -I/usr/include/lua5.2
	;;

#Delete the so file
clean) rm -f visilua.so
	;;

esac
