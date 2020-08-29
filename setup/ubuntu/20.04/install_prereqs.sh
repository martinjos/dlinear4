#!/bin/bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt-get update

apt-get install -y --no-install-recommends $(tr '\n' ' ' <<EOF
bison
flex
g++
libgmp-dev
python3-minimal
make
autoconf
automake
libtool
EOF
)
