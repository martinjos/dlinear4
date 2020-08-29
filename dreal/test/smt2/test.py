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
dreal = sys.argv[1]

# 2nd Argument: smt2 formula name
smt2 = sys.argv[2]

# 3rd Argument: smt2 expected output
expected_output_filename = sys.argv[3]

# 4rd Argument: qsopt_ex lib directory
qsoptex_lib_dir = sys.argv[4]

# 5th Argument: LP solver
lp_solver = sys.argv[5]
assert lp_solver in ("soplex", "qsoptex")

# 6th Argument: simplex phase
phase = sys.argv[6]
assert phase in ("1", "2")

# 7th Argument: SoPlex enabled?
soplex_enabled = sys.argv[7]
if soplex_enabled != "True" and lp_solver == "soplex":
    print("SoPlex not enabled - skipping test")
    sys.exit(0)

options = sys.argv[8:]

options = ["--simplex-sat-phase", phase] + options

options = ["--lp-solver", lp_solver] + options

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
