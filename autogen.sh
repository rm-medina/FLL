#!/bin/sh

aclocal --install -I m4 &&
    autoheader &&
    libtoolize -c --automake --force &&
    autoconf &&
    automake --add-missing --copy &&

x86_64:
    ./configure --with-opencv=/usr/ --enable-debug=full --prefix=/home/rmedina/FLL/target/x86_64/usr/local/fll/ "$@"
#    ./configure --enable-doc --with-opencv=/usr/ "$@"

#arm:
#	./configure --host=arm-linux-gnueabi --build=x86_64 --with-opencv=/home/rmedina/FLL/target/arm/usr/local/opencv/ --enable-debug=full --prefix=/home/rmedina/FLL/target/arm/usr/local/fll/ "$@"
