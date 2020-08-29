dLinear: A Delta-Complete SMT Solver for Linear Real Arithmetic

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


How to Build
============

Install Prerequisites (Ubuntu 20.04, 18.04, 16.04)
--------------------------------------------------

Install the Ubuntu-provided prerequisites: [make](https://www.gnu.org/software/make), [autoconf](https://www.gnu.org/software/autoconf), [automake](https://www.gnu.org/software/automake), [libtool](https://www.gnu.org/software/libtool), [bison](https://www.gnu.org/software/bison), [flex](https://www.gnu.org/software/flex), [GMP](https://gmplib.org/), [Python 3](https://www.python.org/), and a C++ compiler (either [g++](https://www.gnu.org/software/gcc) or [Clang](https://clang.llvm.org/)).
If you are using Ubuntu 20.04, 18.04, or 16.04, you can install these (including `g++`) by running the following commands (or `sudo setup/ubuntu/<version>/install_prereqs.sh`):

```bash
sudo apt-get update
sudo apt-get install bison flex libgmp-dev python3-minimal make autoconf automake libtool
sudo apt-get install g++  # Or: sudo apt-get install clang
```

Install [bazel](https://bazel.build) (a build system by Google, also used by [dReal4](https://github.com/dreal/dreal4)).
Versions 3.1.0 and 3.4.1 have been tested, but later versions should also work.
Instructions are provided on the website.
Alternatively, you can install version 3.4.1 by running `sudo setup/ubuntu/install_bazel.sh` - however, this will not set up the apt repository, so you will not get updates.


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

