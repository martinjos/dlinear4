#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
import subprocess
import difflib

#print("test.py: argv = {!r}".format(sys.argv))

# 1st Argument: dreal path
#               need to check if it exists
dreal = sys.argv.pop(1)

# 2nd Argument: smt2 formula name
smt2 = sys.argv.pop(1)

# 3rd Argument: qsopt_ex lib directory
qsoptex_lib_dir = sys.argv.pop(1)

# 4th Argument: LP solver
lp_solver = sys.argv.pop(1)
assert lp_solver in ("soplex", "qsoptex")

# 5th Argument: simplex phase
phase = sys.argv.pop(1)
assert phase in ("1", "2")

# 6th Argument: SoPlex enabled?
soplex_enabled = sys.argv.pop(1)
if soplex_enabled != "True" and lp_solver == "soplex":
    print("SoPlex not enabled - skipping test")
    sys.exit(0)

# 7th Argument: continuous mode
cont_mode = sys.argv.pop(1)
assert lp_solver == "qsoptex" or cont_mode == "N"

# Remaining Arguments: options to pass through to solver
options = sys.argv[1:]

options = ["--simplex-sat-phase", phase] + options

options = ["--lp-solver", lp_solver] + options

if cont_mode != "N":
    options = ["--continuous-output"] + options
    if cont_mode == "X":
        options = ["--exhaustive"] + options

# Use .expected if present, otherwise look for .expected_phase_*.
expected_output_filename = smt2 + '.expected'
if cont_mode != "N":
    expected_output_filename += '_continuous'
if not os.path.exists(expected_output_filename):
    expected_output_filename += '_phase_{}'.format(phase)
if cont_mode != "N" and not os.path.exists(expected_output_filename):
    print("No reference file in continuous mode - skipping test")
    sys.exit(0)

with open(expected_output_filename, "r") as myfile:
    expected_output = myfile.read().strip().splitlines()

try:
    # 1. Run dReal with smt2 file
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] += ":" + qsoptex_lib_dir
    output = subprocess.check_output([dreal, smt2] + options,
                                     env=env).decode('UTF-8')
    output = output.splitlines()
    print(output)
    # 2. Compare the output with expected output
    diff_result = list(
        difflib.unified_diff(output,
                             expected_output,
                             fromfile='output',
                             tofile='expected output',
                             lineterm=''))
    if diff_result:
        # 3. They are not the same, show the diff.
        for line in diff_result:
            print(line)
        sys.exit(1)
    else:
        # 4. They are the same.
        sys.exit(0)

except subprocess.CalledProcessError as grepexc:
    print("error code", grepexc.returncode, grepexc.output)
    sys.exit(grepexc.returncode)
