#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
import subprocess
import difflib

# 1st Argument: dreal path
#               need to check if it exists
dreal = sys.argv[1]

# 2nd Argument: smt2 formula name
smt2 = sys.argv[2]

# 3rd Argument: smt2 expected output
expected_output_filename = sys.argv[3]

# 4rd Argument: qsopt_ex lib directory
qsoptex_lib_dir = sys.argv[4]

# 5th Argument: simplex phase
phase = int(sys.argv[5])
assert phase in (1, 2)

options = sys.argv[6:]

if phase == 1:
    options = ["--phase-one-simplex"] + options

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
