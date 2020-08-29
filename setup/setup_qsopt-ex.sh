#!/bin/bash

cd "$(dirname "$0")/.."

install_dir="$(pwd)"/install/qsopt-ex
mkdir -p "$install_dir"

git submodule init
git submodule update

cd submodules/qsopt-ex
./bootstrap

mkdir -p build
cd build

../configure --prefix="$install_dir"
make -j4
make install
