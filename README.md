dLinear: A Delta-Complete SMT Solver for Linear Real Arithmetic

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


How to Build
============

Install Prerequisites (Ubuntu 20.04)
------------------------------------

Install the Ubuntu-provided prerequisites: [autoconf](https://www.gnu.org/software/autoconf), [automake](https://www.gnu.org/software/automake), [libtool](https://www.gnu.org/software/libtool), [bison](https://www.gnu.org/software/bison), [flex](https://www.gnu.org/software/flex), [GMP](https://gmplib.org/), [python2.7](https://www.python.org/downloads/release/python-2714/), and a C++ compiler of your choice ([g++](https://www.gnu.org/software/gcc) is recommended).
If you are using Ubuntu 20.04, you can install these using the following commands:

```bash
sudo apt update
sudo apt install autoconf automake libtool bison libfl-dev libgmp-dev
sudo apt install python2-minimal  # If you already have Python 2, you can skip this
sudo apt install g++              # If you already have a C++ compiler, you can skip this
```

Install [bazel](https://bazel.build) (a build system by Google, also used by [dReal4](https://github.com/dreal/dreal4)).
Instructions are provided on the website.
Version 3.1.0 has been tested, but later versions should also work.


Install the qsopt-ex fork
-------------------------

Get the sources.
These should be cloned beneath a common parent directory.

```bash
git clone https://github.com/martinjos/dlinear4.git
git clone https://github.com/martinjos/qsopt-ex.git  # A fork of Debian/Ubuntu version 2.5.10.3-2
```

Build and install `qsopt-ex` using the above repository, setting the install prefix to `dlinear4/qsopt-ex`, as follows:

```bash
cd qsopt-ex
./bootstrap
mkdir build && cd build
../configure --prefix=$(pwd)/../../dlinear4/qsopt-ex
make -j4      # -j4 is optional (number of parallel build processes)
make install  # sudo NOT required
cd ../..
```


Build and Test
--------------

```bash
cd dlinear4
bazel build //...
bazel test //...        # Run all tests
./dlinear.sh <smt2_file>  # Run .smt2 file
```

By default, it builds a release build. To build a debug-build, run
`bazel build //... -c dbg`. In macOS, pass `--apple_generate_dsym` to
allow lldb/gdb to show symbols.

Bazel uses the system default compiler. To use a specific compiler,
set up `CC` environment variable. For example, `CC=gcc-8.0 bazel build
//...`.

Note that although the program can be run using the script `dlinear.sh`, the underlying binary executable file is currently still called `dreal`, for historical reasons.
This will probably change in future.

