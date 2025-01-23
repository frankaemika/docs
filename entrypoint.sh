#!/bin/bash

git submodule update --init --recursive && \
make html

if [ "$#" -gt 0 ]; then
  exec "$@"
fi
