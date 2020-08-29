#!/bin/bash
set -euxo pipefail

echo 'Not implemented' >&2
exit 1

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

brew tap dreal/dreal
brew update
brew install dreal
