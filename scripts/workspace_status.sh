#!/usr/bin/env bash

echo "STABLE_REPOSITORY_STATUS $(git describe --tags --dirty 2>/dev/null)"
