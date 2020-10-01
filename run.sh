#!/bin/bash

here="$(dirname "$0")"
maybe_gdb=

if [ "$1" = "--gdb" ]; then
    maybe_gdb="gdb --args"
    shift
fi

export LD_LIBRARY_PATH="$here"/install/qsopt-ex/lib:$LD_LIBRARY_PATH

$maybe_gdb "$@"
