#!/bin/bash
if [ $# -ge 1 ]; then
  version=$(git describe --all --long --dirty)
  version="$version $(date '+%Y-%m-%d %H:%M:%S') $(git config user.name)"
  echo "Got version string : $version"
  echo $version > $1
else
  echo "error arg"
fi
