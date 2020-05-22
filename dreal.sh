#!/bin/bash

here="$(dirname "$0")"

export LD_LIBRARY_PATH="$here"/qsopt-ex/lib:$LD_LIBRARY_PATH

"$here"/bazel-bin/dreal/dreal "$@"
