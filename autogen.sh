#!/bin/sh

aclocal --install -I m4 &&
    autoheader &&
    libtoolize -c --automake --force &&
    autoconf &&
    automake --add-missing --copy &&
    ./configure --with-opencv=/usr/ --enable-debug=full  "$@"
#    ./configure --enable-doc --with-opencv=/usr/ "$@"
